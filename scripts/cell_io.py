#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""cell_io: bridges the gripper cell to the PLC using plain std_msgs/Bool topics that match the
plc_bridge contract (github.com/bb53192/plc-ros2-bridge).

  PLC -> cell:   /plc/belt_run     (Bool)  OPC UA  true = run belt, false = stop
                 /plc/robot_start  (Bool)  OPC UA  false->true edge = run ONE pick-and-place cycle
  cell -> PLC:   /plc/write/robot_busy (Bool)  OPC UA (write)  true while a cycle is running
                 /plc/write/cycle_done (Bool)  OPC UA (write)  true after a cycle, until next start
                 /sensors/part_present (Bool)  Modbus discrete input  part at the belt end stop

The handshake rides OPC UA (bridge writes writable OpenPLC vars from /plc/write/<name>); only the
part_present sensor stays on Modbus (/sensors/<name> -> modbus_sensor_bridge discrete input).
The belt command goes out on /conveyor/belt_cmd (Float64), bridged to gz by gz_bridge.yaml.
The pick-and-place cycle uses MoveIt /compute_ik + arm_controller/gripper_controller directly
(same method as crx_pick_place.py), run in a worker thread so the I/O keeps updating live.
A fresh workpiece is respawned at the belt feed end after each cycle (gz set_pose).
"""
from __future__ import annotations

import math
import pathlib
import subprocess
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["J1", "J2", "J3", "J4", "J5", "J6"]
GRIP = ["robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint"]


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _yaw_follow_quat(base_quat, dz):
    """base_quat rotated about world Z by dz. Per waypoint with dz = azimuth(wp) - azimuth(pick),
    it keeps the arm posture across the pick->place swing so J1 carries it (the wrist doesn't
    unwind to hold a fixed world orientation). Arm base is the world origin here."""
    h = dz / 2.0
    return _quat_mul((0.0, 0.0, math.sin(h), math.cos(h)), base_quat)


class CellIO(Node):
    def __init__(self) -> None:
        super().__init__("cell_io")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("belt_speed", -0.15)
        self.declare_parameter("world", "gripper_cell")
        self.declare_parameter("feed_pose", [1.75, 0.35, 0.58])  # belt feed end respawn
        # gz contact sensor only publishes WHILE touching; infer "gone" from silence.
        self.declare_parameter("present_timeout", 0.5)

        wp = self.get_parameter("waypoint_file").value
        path = (
            pathlib.Path(wp).expanduser()
            if wp
            else pathlib.Path(get_package_share_directory("ros_plc_sim"))
            / "config" / "crx_pick_place_waypoints.yaml"
        )
        d = yaml.safe_load(path.read_text())
        self.frame = d.get("planning_frame", "world")
        self.off = float(d["flange_tcp_offset"])
        o = d["orientation"]
        self.quat = (o["x"], o["y"], o["z"], o["w"])
        self.move_time = float(d.get("move_time", 4.0))
        self.grip_time = float(d.get("grip_time", 2.0))
        self.home = d["home"]
        self.tcp = d["tcp"]
        self.g_open = float(d["gripper"]["open"])
        self.g_closed = float(d["gripper"]["closed"])
        p = self.tcp["pick"]
        self.az_pick = math.atan2(float(p["y"]), float(p["x"]))  # base at world origin
        self._ref = None            # cached belt-side reference posture
        self._last = None           # last commanded joints (ARM order) for short-way unwrapping
        self.belt_speed = float(self.get_parameter("belt_speed").value)
        self.world = self.get_parameter("world").value
        self.feed = list(self.get_parameter("feed_pose").value)
        self.present_timeout = float(self.get_parameter("present_timeout").value)

        # ---- state
        self._belt_run = False
        self._part_present = False
        self._robot_busy = False
        self._cycle_done = False
        self._start_prev = False
        self._last_contact = 0.0  # monotonic time of last non-empty contact msg
        self._lock = threading.Lock()

        cg = ReentrantCallbackGroup()
        # ---- PLC-facing I/O
        self.pub_belt = self.create_publisher(Float64, "/conveyor/belt_cmd", 10)
        self.pub_present = self.create_publisher(Bool, "/sensors/part_present", 10)  # Modbus
        self.pub_busy = self.create_publisher(Bool, "/plc/write/robot_busy", 10)      # OPC UA write
        self.pub_done = self.create_publisher(Bool, "/plc/write/cycle_done", 10)      # OPC UA write
        self.create_subscription(Bool, "/plc/belt_run", self._on_belt_run, 10, callback_group=cg)
        self.create_subscription(Bool, "/plc/robot_start", self._on_robot_start, 10, callback_group=cg)
        self.create_subscription(
            Contacts, "/conveyor_end_sensor/contacts", self._on_contacts, 10, callback_group=cg
        )

        # ---- motion clients
        self.ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=cg)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory", callback_group=cg)
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory", callback_group=cg)

        self.create_timer(0.2, self._tick, callback_group=cg)
        self.get_logger().info("cell_io up: /plc/belt_run, /plc/robot_start -> /sensors/*")

    # ------------------------------------------------------------------ I/O
    def _on_belt_run(self, msg: Bool) -> None:
        self._belt_run = bool(msg.data)

    def _on_contacts(self, msg: Contacts) -> None:
        # gz publishes this topic only while contact exists; stamp the time and let
        # _tick clear part_present once messages stop arriving (part picked away).
        if len(msg.contacts) > 0:
            self._last_contact = time.monotonic()

    def _on_robot_start(self, msg: Bool) -> None:
        start = bool(msg.data)
        if start and not self._start_prev and not self._robot_busy:
            self._start_prev = True
            threading.Thread(target=self._run_cycle, daemon=True).start()
        elif not start:
            self._start_prev = False

    def _tick(self) -> None:
        # part_present is true only while contacts keep arriving (see _on_contacts)
        self._part_present = (time.monotonic() - self._last_contact) < self.present_timeout
        self.pub_belt.publish(Float64(data=(self.belt_speed if self._belt_run else 0.0)))
        self.pub_present.publish(Bool(data=self._part_present))
        self.pub_busy.publish(Bool(data=self._robot_busy))
        self.pub_done.publish(Bool(data=self._cycle_done))

    # ------------------------------------------------------------------ motion
    def _await(self, future, timeout=60.0):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise RuntimeError("timed out waiting on future")
        return future.result()

    def _solve_raw(self, x, y, z, quat, seed=None) -> list:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.ik_link_name = "flange"
        if seed is not None:
            js = JointState()
            js.name = list(ARM)
            js.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = js
            req.ik_request.robot_state.is_diff = False
        else:
            req.ik_request.robot_state.is_diff = True
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        req.ik_request.pose_stamped = ps
        r = self._await(self.ik.call_async(req), timeout=8.0)
        if not r or r.error_code.val != 1:
            raise RuntimeError("IK failed")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def _ref_posture(self) -> list:
        if self._ref is None:
            p = self.tcp["pick"]
            self._ref = self._solve_raw(p["x"], p["y"], float(p["z"]) + self.off, self.quat)
        return self._ref

    @staticmethod
    def _unwrap(target, ref) -> float:
        while target - ref > math.pi:
            target -= 2 * math.pi
        while target - ref < -math.pi:
            target += 2 * math.pi
        return target

    def _solve(self, name: str) -> dict:
        # Orientation follows the base yaw and the IK is seeded with the reference posture at a
        # pre-rotated J1, so J1 (not the wrist) carries the pick<->place swing; unwrap to the
        # current joints so the controller takes the short way.
        t = self.tcp[name]
        dz = math.atan2(float(t["y"]), float(t["x"])) - self.az_pick
        quat = _yaw_follow_quat(self.quat, dz)
        ref = self._ref_posture()
        seed = [ref[0] + dz] + list(ref[1:])
        joints = self._solve_raw(t["x"], t["y"], float(t["z"]) + self.off, quat, seed=seed)
        base = self._last if self._last is not None else seed
        joints = [self._unwrap(j, b) for j, b in zip(joints, base)]
        self._last = joints
        return {ARM[i]: joints[i] for i in range(len(ARM))}

    def _traj(self, client, names, positions, secs):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(secs)
        pt.time_from_start.nanosec = int((secs % 1) * 1e9)
        g.trajectory.points = [pt]
        gh = self._await(client.send_goal_async(g))
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        self._await(gh.get_result_async())
        time.sleep(0.4)

    def _arm_to(self, joints, secs=None):
        self._traj(self.arm, ARM, [joints[j] for j in ARM], secs or self.move_time)

    def _grip_to(self, pos):
        self._traj(self.grip, GRIP, [pos, pos], self.grip_time)
        time.sleep(0.3)

    def _go(self, name):
        self.get_logger().info(f"-> {name}")
        self._arm_to(self._solve(name))

    def _respawn(self) -> None:
        x, y, z = self.feed
        req = f'name: "workpiece_1", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1.0}}'
        subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", req],
            capture_output=True, timeout=10,
        )
        self.get_logger().info("respawned workpiece at feed end")

    def _run_cycle(self) -> None:
        with self._lock:
            self._robot_busy = True
            self._cycle_done = False
        try:
            self.get_logger().info("cycle: start")
            self._arm_to(self.home)
            self._last = [self.home[j] for j in ARM]   # unwrap the first waypoint relative to home
            self._grip_to(self.g_open)
            self._go("pick_approach")
            self._go("pick")
            self._grip_to(self.g_closed)
            self._go("lift")
            self._go("place_approach")
            self._go("place")
            self._grip_to(self.g_open)
            self._go("place_approach")
            self._arm_to(self.home)
            self._respawn()
            self.get_logger().info("cycle: done")
            with self._lock:
                self._cycle_done = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cycle failed: {exc}")
        finally:
            with self._lock:
                self._robot_busy = False


def main() -> None:
    rclpy.init()
    node = CellIO()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
