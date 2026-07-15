#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""sort_cell: orchestrates the two-arm color-sorting line.

One part at a time rides the belt toward the arms. It stops at station B (green, upstream); an
overhead camera classifies it. If GREEN, the green arm picks it into the green box. Otherwise
gate B raises and the part passes under to station A (blue); if BLUE, the blue arm picks it into
the blue box; otherwise gate A raises and the part falls off the belt end into the unsorted bin.
A fresh, randomly-colored part is then spawned at the feed end and the cycle repeats.

Motion reuses the single-arm approach (MoveIt /compute_ik + FollowJointTrajectory driven
directly), generalized into an `Arm` helper parameterized by joint prefix / group / controllers.

  ros2 run ros_plc_sim sort_cell.py            # needs the cell + move_group up
"""
from __future__ import annotations

import math
import pathlib
import random
import re
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
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

# Part colors -> RGB. Weighted so the two sorted colors dominate the demo.
COLORS = {
    "blue": (0.10, 0.20, 0.85),
    "green": (0.10, 0.70, 0.15),
    "red": (0.85, 0.10, 0.10),
    "yellow": (0.90, 0.85, 0.10),
}
SPAWN_WEIGHTS = [("blue", 0.35), ("green", 0.35), ("red", 0.15), ("yellow", 0.15)]

GATE_OPEN = 0.12
GATE_CLOSED = 0.0


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _yaw_follow_quat(base_quat, dz):
    """base_quat rotated about world Z by dz. Applied per waypoint with dz = azimuth(wp) -
    azimuth(pick), it keeps the arm's posture across the pick->place swing so J1 (base yaw)
    carries the motion instead of the wrist unwinding to hold a fixed world orientation."""
    h = dz / 2.0
    return _quat_mul((0.0, 0.0, math.sin(h), math.cos(h)), base_quat)


class Arm:
    """One CRX arm: /compute_ik -> arm_controller / gripper_controller (direct trajectories)."""

    def __init__(self, node: Node, prefix: str, group: str, ik_link: str,
                 arm_action: str, grip_action: str, wp: dict, shared: dict, cg,
                 base_xy=(0.0, 0.0)) -> None:
        self.node = node
        self.log = node.get_logger()
        self.group = group
        self.ik_link = ik_link
        self.base_x, self.base_y = base_xy
        self.arm_joints = [f"{prefix}J{i}" for i in range(1, 7)]
        self.grip_joints = [f"{prefix}robotiq_85_left_knuckle_joint",
                            f"{prefix}robotiq_85_right_knuckle_joint"]
        self.wp = wp
        self.frame = shared["frame"]
        self.off = shared["off"]
        self.quat = shared["quat"]
        self.move_time = shared["move_time"]
        self.grip_time = shared["grip_time"]
        self.home = [shared["home"][f"J{i}"] for i in range(1, 7)]
        self.g_open = shared["g_open"]
        self.g_closed = shared["g_closed"]
        # azimuth of the grasp point from the arm base; other waypoints rotate the tool
        # orientation about world Z relative to this, so the base yaw (J1) carries the swing.
        p = wp["pick"]
        self.az_pick = math.atan2(float(p["y"]) - self.base_y, float(p["x"]) - self.base_x)
        self._ref = None            # cached belt-side reference posture (see _ref_posture)
        self._last_joints = None    # last commanded joints (for short-way unwrapping)

        self.ik = node.create_client(GetPositionIK, "/compute_ik", callback_group=cg)
        self.arm = ActionClient(node, FollowJointTrajectory, arm_action, callback_group=cg)
        self.grip = ActionClient(node, FollowJointTrajectory, grip_action, callback_group=cg)

    def wait_ready(self, timeout=20.0) -> None:
        if not self.ik.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_ik unavailable (move_group up?)")
        self.arm.wait_for_server(timeout_sec=timeout)
        self.grip.wait_for_server(timeout_sec=timeout)

    def _await(self, future, timeout=60.0):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise RuntimeError("timed out waiting on future")
        return future.result()

    def _solve_raw(self, x, y, z, quat, seed=None) -> list:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.ik_link
        if seed is not None:
            js = JointState()
            js.name = list(self.arm_joints)
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
            raise RuntimeError(f"IK failed (group {self.group})")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in self.arm_joints]

    def _ref_posture(self) -> list:
        """Natural belt-side posture at the grasp point (cached), used to seed every waypoint so
        the base yaw (J1) — not the wrist — carries the pick<->place swing."""
        if self._ref is None:
            p = self.wp["pick"]
            self._ref = self._solve_raw(p["x"], p["y"], float(p["z"]) + self.off, self.quat)
        return self._ref

    @staticmethod
    def _unwrap(target, ref) -> float:
        while target - ref > math.pi:
            target -= 2 * math.pi
        while target - ref < -math.pi:
            target += 2 * math.pi
        return target

    def _solve(self, name: str) -> list:
        t = self.wp[name]
        dz = math.atan2(float(t["y"]) - self.base_y, float(t["x"]) - self.base_x) - self.az_pick
        quat = _yaw_follow_quat(self.quat, dz)                 # orientation follows the base yaw
        ref = self._ref_posture()
        seed = [ref[0] + dz] + list(ref[1:])                   # reference posture with J1 pre-rotated
        joints = self._solve_raw(t["x"], t["y"], float(t["z"]) + self.off, quat, seed=seed)
        base = self._last_joints if self._last_joints is not None else seed
        joints = [self._unwrap(j, b) for j, b in zip(joints, base)]  # shortest way from current
        self._last_joints = joints
        return joints

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
        time.sleep(0.15)   # brief settle between moves

    def _arm_to(self, positions, secs=None):
        self._traj(self.arm, self.arm_joints, positions, secs or self.move_time)

    def _grip_to(self, pos):
        self._traj(self.grip, self.grip_joints, [pos, pos], self.grip_time)
        time.sleep(0.3)

    def _go(self, name):
        self.log.info(f"[{self.group}] -> {name}")
        self._arm_to(self._solve(name))

    def pick_place(self) -> None:
        """One full pick-and-place cycle into this arm's box."""
        self._arm_to(self.home)
        self._last_joints = list(self.home)   # unwrap the first waypoint relative to home
        self._grip_to(self.g_open)
        self._go("pick_approach")
        self._go("pick")
        self._grip_to(self.g_closed)
        self._go("lift")
        self._place_tail()

    def _place_tail(self) -> None:
        """From holding the part above the pick zone, carry it into the box and go home."""
        self._go("place_approach")
        self._go("place")
        self._grip_to(self.g_open)
        self._go("place_approach")
        self._arm_to(self.home)
        self._last_joints = list(self.home)

    # ------------------------------------------------------------------ on-the-fly intercept
    def _solve_xyz(self, x, y, z) -> list:
        """IK for a world TCP (x, y, z) with the yaw-following grasp orientation, seeded+unwrapped
        from the last commanded joints (same convention as _solve). Updates _last_joints."""
        dz = math.atan2(y - self.base_y, x - self.base_x) - self.az_pick
        quat = _yaw_follow_quat(self.quat, dz)
        ref = self._ref_posture()
        seed = self._last_joints if self._last_joints is not None else [ref[0] + dz] + list(ref[1:])
        j = self._solve_raw(x, y, z + self.off, quat, seed=seed)
        j = [self._unwrap(a, b) for a, b in zip(j, seed)]
        self._last_joints = j
        return j

    def move_to(self, x, y, z, secs=None) -> None:
        self._arm_to(self._solve_xyz(x, y, z), secs)

    def build_intercept(self, itc: dict, v: float, x0: float):
        """Compute the co-move arm trajectory for grabbing a part that is at x0 (and moving -X at
        speed v) when the trajectory STARTS. Returns (points, t_close) where points are
        (positions, t_from_start) and t_close is when to start closing the gripper (arm at grasp
        height, co-moving)."""
        y, za, zg = float(itc["y"]), float(itc["z_approach"]), float(itc["z_grasp"])
        td, tgr, tl = float(itc["descend_time"]), float(itc["grasp_time"]), float(itc["lift_time"])
        x_grasp_end = x0 - v * (td + tgr)
        # arm is pre-positioned at (x0, za). Descend while co-moving -X at v to meet the part at
        # grasp height, hold (gripper closes), then lift STRAIGHT UP (x held -- the part is now in
        # the gripper, so no need to keep chasing the belt, and it keeps the wrist out over the base).
        pts = []
        for xc, zc, tc in [
            (x0 - v * td,   zg, td),                 # grasp start (end of descend)
            (x_grasp_end,   zg, td + tgr),           # grasp end (part held)
            (x_grasp_end,   za, td + tgr + tl),      # lift straight up
        ]:
            pts.append((self._solve_xyz(xc, y, zc), tc))
        return pts, td

    def fire_intercept(self, pts, t_close) -> None:
        """Send the pre-computed co-move trajectory and close the gripper mid-glide. The gripper
        goal holds open until t_close then closes, on the SAME sim clock as the arm, so the two
        stay synced regardless of real-time factor."""
        ga = FollowJointTrajectory.Goal()
        ga.trajectory.joint_names = self.arm_joints
        for positions, tc in pts:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in positions]
            p.time_from_start.sec = int(tc)
            p.time_from_start.nanosec = int((tc % 1) * 1e9)
            ga.trajectory.points.append(p)
        arm_gh = self._await(self.arm.send_goal_async(ga))
        if arm_gh is None or not arm_gh.accepted:
            raise RuntimeError("intercept trajectory rejected")
        # gripper: start closing DURING the last part of the descent so the jaws are already
        # closing as they reach the moving cube (less chance of knocking it), and finish soon after
        # grasp height so there is a firm-grip dwell before the lift.
        tc1 = max(0.1, t_close - 0.4)
        tc2 = t_close + 0.4
        gg = FollowJointTrajectory.Goal()
        gg.trajectory.joint_names = self.grip_joints
        for pos, tc in [(self.g_open, tc1), (self.g_closed, tc2)]:
            p = JointTrajectoryPoint()
            p.positions = [float(pos), float(pos)]
            p.time_from_start.sec = int(tc)
            p.time_from_start.nanosec = int((tc % 1) * 1e9)
            gg.trajectory.points.append(p)
        self.grip.send_goal_async(gg)                 # fire and forget (runs on sim clock)
        self._await(arm_gh.get_result_async())        # arm co-move (incl. lift) finished
        self._last_joints = pts[-1][0]
        time.sleep(0.3)


class SortCell(Node):
    def __init__(self) -> None:
        super().__init__("sort_cell")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("belt_speed", -0.25)
        self.declare_parameter("world", "sorting_cell")
        self.declare_parameter("feed_pose", [2.40, 0.35, 0.58])
        self.declare_parameter("present_timeout", 0.5)
        self.declare_parameter("settle_time", 1.5)
        self.declare_parameter("force_color", "")  # debug: force every part's color (blue/green/red/yellow)
        # Experimental: green arm grabs the part in motion (belt never stops). The intercept aligns
        # well but the dynamic grasp of the 4 cm cube isn't reliable yet (needs closed-loop feedback),
        # so this defaults OFF -> the reliable static pick is used. Enable with on_the_fly:=true.
        self.declare_parameter("on_the_fly", False)

        wp = self.get_parameter("waypoint_file").value
        path = (pathlib.Path(wp).expanduser() if wp
                else pathlib.Path(get_package_share_directory("ros_plc_sim"))
                / "config" / "dual_pick_place_waypoints.yaml")
        d = yaml.safe_load(path.read_text())
        shared = {
            "frame": d.get("planning_frame", "world"),
            "off": float(d["flange_tcp_offset"]),
            "quat": (d["orientation"]["x"], d["orientation"]["y"],
                     d["orientation"]["z"], d["orientation"]["w"]),
            "move_time": float(d.get("move_time", 4.0)),
            "grip_time": float(d.get("grip_time", 2.0)),
            "home": d["home"],
            "g_open": float(d["gripper"]["open"]),
            "g_closed": float(d["gripper"]["closed"]),
        }
        self.belt_speed = float(self.get_parameter("belt_speed").value)
        self.world = self.get_parameter("world").value
        self.feed = list(self.get_parameter("feed_pose").value)
        self.present_timeout = float(self.get_parameter("present_timeout").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.force_color = str(self.get_parameter("force_color").value)
        self.on_the_fly = bool(self.get_parameter("on_the_fly").value)
        self.itc = d.get("green_intercept", {})

        cg = ReentrantCallbackGroup()
        # base_xy = each arm's world mount pose (see dual_crx_gz.urdf.xacro).
        self.blue = Arm(self, "blue_", "blue_manipulator", "blue_flange",
                        "/blue_arm_controller/follow_joint_trajectory",
                        "/blue_gripper_controller/follow_joint_trajectory", d["blue"], shared, cg,
                        base_xy=(0.0, 0.0))
        self.green = Arm(self, "green_", "green_manipulator", "green_flange",
                         "/green_arm_controller/follow_joint_trajectory",
                         "/green_gripper_controller/follow_joint_trajectory", d["green"], shared, cg,
                         base_xy=(1.0, 0.0))

        # ---- state
        self._belt_run = True
        self._belt_override = None   # temporary belt speed (m/s) during an on-the-fly grasp crawl
        self._counter = 0
        self._part_name = None
        self._gate_a = GATE_CLOSED
        self._gate_b = GATE_CLOSED
        self._last_contact = {"a": 0.0, "b": 0.0}
        self._color = {"a": "none", "b": "none"}
        self._part_hist = []   # recent (monotonic_t, x) of the active part, for tracking

        # ---- I/O
        self.pub_belt = self.create_publisher(Float64, "/conveyor/belt_cmd", 10)
        self.pub_gate_a = self.create_publisher(Float64, "/gate_a/cmd_pos", 10)
        self.pub_gate_b = self.create_publisher(Float64, "/gate_b/cmd_pos", 10)
        self.create_subscription(Contacts, "/station_a/contacts",
                                 lambda m: self._on_contact("a", m), 10, callback_group=cg)
        self.create_subscription(Contacts, "/station_b/contacts",
                                 lambda m: self._on_contact("b", m), 10, callback_group=cg)
        self.create_subscription(String, "/station_a/part_color",
                                 lambda m: self._color.__setitem__("a", m.data), 10, callback_group=cg)
        self.create_subscription(String, "/station_b/part_color",
                                 lambda m: self._color.__setitem__("b", m.data), 10, callback_group=cg)
        self.create_subscription(TFMessage, "/model_poses", self._on_poses, 10, callback_group=cg)
        self.create_timer(0.1, self._tick, callback_group=cg)  # hold belt + gate commands @10 Hz
        threading.Thread(target=self._run, daemon=True).start()
        self.get_logger().info("sort_cell up")

    # ------------------------------------------------------------------ I/O
    def _on_contact(self, station: str, msg: Contacts) -> None:
        if len(msg.contacts) > 0:
            self._last_contact[station] = time.monotonic()

    def _present(self, station: str) -> bool:
        return (time.monotonic() - self._last_contact[station]) < self.present_timeout

    def _on_poses(self, msg: TFMessage) -> None:
        name = self._part_name
        if not name:
            return
        for tf in msg.transforms:
            if tf.child_frame_id == name:
                self._part_hist.append((time.monotonic(), tf.transform.translation.x))
                if len(self._part_hist) > 8:
                    self._part_hist.pop(0)
                break

    def _part_x(self):
        return self._part_hist[-1][1] if self._part_hist else None

    def _gz_part_x(self):
        """Query the active part's world x directly from gz (reliable here, unlike /model_poses).
        Runs in the sequencer worker thread, so the blocking subprocess is fine. None on failure."""
        if not self._part_name:
            return None
        try:
            out = subprocess.run(["gz", "model", "-m", self._part_name, "-p"],
                                 capture_output=True, timeout=2, text=True).stdout
        except Exception:  # noqa: BLE001
            return None
        m = re.search(r"XYZ[^\n]*\n\s*\[\s*([-\d.eE]+)", out)
        return float(m.group(1)) if m else None

    def _part_speed(self):
        """Magnitude of the part's -X motion (m/s), from the recent pose history; None if unknown."""
        if len(self._part_hist) < 2:
            return None
        (t0, x0), (t1, x1) = self._part_hist[0], self._part_hist[-1]
        return (x0 - x1) / (t1 - t0) if t1 > t0 else 0.0

    def _tick(self) -> None:
        # _belt_override (when set) temporarily replaces the belt speed -- used to CRAWL the belt
        # during an on-the-fly grasp (belt still moves, just slowly, so the window is wide).
        speed = self._belt_override if self._belt_override is not None else self.belt_speed
        self.pub_belt.publish(Float64(data=(speed if self._belt_run else 0.0)))
        self.pub_gate_a.publish(Float64(data=self._gate_a))
        self.pub_gate_b.publish(Float64(data=self._gate_b))

    # ------------------------------------------------------------------ helpers
    def _gz(self, service: str, reqtype: str, req: str) -> None:
        subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/{service}",
             "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", req],
            capture_output=True, timeout=10,
        )

    def _spawn_part(self, color: str) -> None:
        r, g, b = COLORS[color]
        x, y, z = self.feed
        self._part_name = f"wp_{self._counter}"
        self._counter += 1
        self._part_hist = []   # fresh position history for the new part
        sdf = (
            f"<sdf version='1.8'><model name='{self._part_name}'><pose>{x} {y} {z} 0 0 0</pose>"
            f"<link name='link'><inertial><mass>0.05</mass>"
            f"<inertia><ixx>2e-5</ixx><iyy>2e-5</iyy><izz>2e-5</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            f"<collision name='c'><geometry><box><size>0.04 0.04 0.04</size></box></geometry>"
            f"<surface><friction><ode><mu>50</mu><mu2>50</mu2></ode></friction></surface></collision>"
            f"<visual name='v'><geometry><box><size>0.04 0.04 0.04</size></box></geometry>"
            f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
            f"</visual></link></model></sdf>"
        )
        self._gz("create", "gz.msgs.EntityFactory", f'sdf: "{sdf}"')

    def _remove_named(self, name: str) -> None:
        self._gz("remove", "gz.msgs.Entity", f'name: "{name}" type: MODEL')

    def _random_color(self) -> str:
        if self.force_color in COLORS:
            return self.force_color
        r = random.random()
        acc = 0.0
        for name, w in SPAWN_WEIGHTS:
            acc += w
            if r <= acc:
                return name
        return SPAWN_WEIGHTS[-1][0]

    def _wait(self, cond, timeout: float, poll=0.1) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if cond():
                return True
            time.sleep(poll)
        return False

    def _read_color(self, station: str) -> str:
        """Discard any stale cached color, then majority-vote over the classifier's FRESH
        readings. Resetting first is essential: under a slow sim the classifier lags, and the
        previous part's color lingering in the cache was misrouting the next part (e.g. a blue
        part read as the previous green). Collect enough agreeing fresh samples, or time out."""
        self._color[station] = "none"
        votes: dict[str, int] = {}
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            c = self._color[station]
            if c and c != "none":
                votes[c] = votes.get(c, 0) + 1
                if sum(votes.values()) >= 8:
                    break
            time.sleep(0.1)
        if not votes:
            return "none"
        return max(votes, key=votes.get)

    # ------------------------------------------------------------------ sequencer
    def _run(self) -> None:
        try:
            self.blue.wait_ready()
            self.green.wait_ready()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"arms not ready: {exc}")
            return
        # start clean: remove the world's default part, both gates closed
        self._remove_named("workpiece_1")
        self._gate_a = GATE_CLOSED
        self._gate_b = GATE_CLOSED
        time.sleep(1.0)

        while rclpy.ok():
            color = self._random_color()
            self.get_logger().info(f"=== new part: {color} ===")
            self._spawn_part(color)

            # The part is classified ONCE, at station B, where it stops cleanly. That single
            # decision routes it the whole way -- we do NOT re-read at station A. Re-reading at A
            # was a second failure point: a blurry/off-center frame there sent a correctly-seen
            # blue part to the unsorted bin.
            if not self._wait(lambda: self._present("b"), timeout=25.0):
                self.get_logger().warn("part never reached station B; removing & retrying")
                self._remove_named(self._part_name); time.sleep(0.5); continue
            time.sleep(self.settle_time)
            color = self._read_color("b")
            if color == "none" and self._present("b"):   # present but unseen -> settle & retry once
                time.sleep(1.0)
                color = self._read_color("b")
            self.get_logger().info(f"station B classifies part as: {color}")

            if color == "green":
                if self.on_the_fly:
                    self._intercept_green()      # belt keeps running; grab the part in motion
                else:
                    self.get_logger().info("-> GREEN arm static pick into green box")
                    self._belt_run = False       # stop the belt so the part is still during grasp
                    self.green.pick_place()
                    self._belt_run = True
                self._settle(); continue

            # not green: open gate B, let the part pass to station A (blue)
            self.get_logger().info("-> not green: opening gate B, part passes to station A")
            self._gate_b = GATE_OPEN
            self._wait(lambda: not self._present("b"), timeout=5.0)   # cleared B
            if not self._wait(lambda: self._present("a"), timeout=15.0):
                self.get_logger().warn("part never reached station A; recovering")
                self._gate_b = GATE_CLOSED; self._settle(); continue
            self._gate_b = GATE_CLOSED
            time.sleep(self.settle_time)

            if color == "blue":
                self.get_logger().info("-> BLUE arm picks into blue box")
                self._belt_run = False          # stop the belt so the part is still during grasp
                self.blue.pick_place()
                self._belt_run = True
                self._settle(); continue

            # neither green nor blue: open gate A, part falls into the unsorted bin
            self.get_logger().info("-> unsorted: opening gate A, part drops into bin")
            self._gate_a = GATE_OPEN
            self._wait(lambda: not self._present("a"), timeout=5.0)
            time.sleep(2.0)
            self._gate_a = GATE_CLOSED
            self._settle()

    def _intercept_green(self) -> None:
        """Grab the green part while it keeps moving on the belt (belt never stops). Classify has
        already happened at gate B; here we pre-position the green arm, release the part, and fire a
        pre-computed velocity-matched trajectory the moment the part reaches the intercept point."""
        itc = self.itc
        g = self.green
        v = float(itc.get("match_speed", 0.03))
        # home + open, then pre-position above the intercept point while the part is still held
        g._arm_to(g.home); g._last_joints = list(g.home)
        g._grip_to(g.g_open)
        g.move_to(float(itc["x_intercept"]), float(itc["y"]), float(itc["z_approach"]))  # ready pose
        # release the part, but CRAWL the belt (it keeps moving, just slowly).
        self.get_logger().info("-> GREEN on-the-fly: crawling belt, releasing part (belt never stops)")
        self._belt_override = float(itc.get("belt_crawl", -0.06))
        self._gate_b = GATE_OPEN
        # POSITION trigger (RTF-independent): poll the part's real x, fire when it reaches x_intercept.
        x_fire = float(itc["x_intercept"])
        t0 = time.monotonic()
        px = None
        while time.monotonic() - t0 < 15.0:
            px = self._gz_part_x()
            if px is not None and px <= x_fire:
                break
            time.sleep(0.04)
        if px is None or px > x_fire:
            self.get_logger().warn("part never reached intercept point; recovering")
            self._gate_b = GATE_CLOSED; self._belt_override = None
            return
        # Build the co-move NOW from the detected position, predicting the part forward by the
        # build+send latency so the trajectory's start position matches where the part will be.
        x0 = px - v * float(itc.get("fire_latency", 0.5))
        tb = time.monotonic()
        pts, t_close = g.build_intercept(itc, v, x0)
        build_s = time.monotonic() - tb
        px_now = self._gz_part_x()   # where the part actually is now (after the build)
        self.get_logger().info(
            f"intercept fire (px={px:.3f} x0={x0:.3f} build={build_s:.2f}s px_after_build={px_now})")
        g.fire_intercept(pts, t_close)              # co-move grasp + lift while the belt crawls
        self._belt_override = None                  # part is off the belt -> back to full belt speed
        self._gate_b = GATE_CLOSED
        g._place_tail()                             # carry the grasped part into the green box

    def _settle(self) -> None:
        # Placed/binned parts are left in place (they accumulate in the boxes/bin).
        time.sleep(1.5)


def main() -> None:
    rclpy.init()
    node = SortCell()
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
