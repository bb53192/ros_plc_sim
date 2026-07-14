#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""One CRX pick-and-place cycle: belt end-stop -> collection box.

Uses MoveIt's /compute_ik to turn tool-down TCP waypoints into joint targets, then drives
arm_controller and gripper_controller (FollowJointTrajectory) directly -- this is deterministic
and descends reliably, unlike arm_api2's cartesian (LIN) mode which silently would not descend
here. Grasp orientation twists the jaws to straddle the part along world Y (clear of the
end-stop lip); grasp height is calibrated in the waypoint file.

Run (cell + move_group up):
  ros2 run ros_plc_sim crx_pick_place.py
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
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


class CrxPickPlace(Node):
    def __init__(self) -> None:
        super().__init__("crx_pick_place")
        self.declare_parameter("waypoint_file", "")
        wp = self.get_parameter("waypoint_file").value
        path = (
            pathlib.Path(wp).expanduser()
            if wp
            else pathlib.Path(get_package_share_directory("ros_plc_sim"))
            / "config"
            / "crx_pick_place_waypoints.yaml"
        )
        if not path.is_file():
            self.get_logger().fatal(f"waypoint_file missing: {path}")
            raise SystemExit(1)
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

        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        if not self.ik.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/compute_ik unavailable (is move_group running?)")
        self.arm.wait_for_server(timeout_sec=15.0)
        self.grip.wait_for_server(timeout_sec=15.0)
        self._run()

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
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if not r or r.error_code.val != 1:
            raise RuntimeError(f"IK failed (error {r.error_code.val if r else None})")
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

    def _arm_to(self, joints: dict, secs: float) -> None:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [float(joints[j]) for j in ARM]
        pt.time_from_start.sec = int(secs)
        pt.time_from_start.nanosec = int((secs % 1) * 1e9)
        g.trajectory.points = [pt]
        fut = self.arm.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("arm goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        time.sleep(0.5)

    def _grip_to(self, pos: float, label: str) -> None:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = GRIP
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start.sec = int(self.grip_time)
        g.trajectory.points = [pt]
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"gripper {label} goal rejected")
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        self.get_logger().info(f"gripper {label}")
        time.sleep(0.7)

    def _go(self, name: str) -> None:
        self.get_logger().info(f"-> {name}")
        self._arm_to(self._solve(name), self.move_time)

    def _run(self) -> None:
        self.get_logger().info("home + open")
        self._arm_to(self.home, self.move_time)
        self._last = [self.home[j] for j in ARM]   # unwrap the first waypoint relative to home
        self._grip_to(self.g_open, "open")
        self._go("pick_approach")
        self._go("pick")
        self._grip_to(self.g_closed, "close")
        self._go("lift")
        self._go("place_approach")
        self._go("place")
        self._grip_to(self.g_open, "open")
        self._go("place_approach")
        self._arm_to(self.home, self.move_time)
        self.get_logger().info("pick-and-place cycle complete")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CrxPickPlace()
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        print(exc, file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
