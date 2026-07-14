#!/usr/bin/env python3
"""AUTO/MANUAL command arbiter for the FANUC CRX gripper cell.

Selects who drives the cell's command signals (belt_run, robot_start):

  AUTO   -> pass through the PLC's commands  (/plc/belt_run, /plc/robot_start)
  MANUAL -> pass through the HMI's commands  (/manual/belt_run, /manual/robot_start)

Output goes to /cmd/belt_run, /cmd/robot_start, which cell_io consumes (launch
remaps cell_io's /plc/* inputs onto /cmd/*). This keeps the plc_bridge invocation
unchanged — it still publishes /plc/belt_run, /plc/robot_start as before.

Mode is switched via the /cell/set_mode service (std_srvs/SetBool: data=True
=> AUTO, data=False => MANUAL) and broadcast on /cell/mode (Bool, True=AUTO)
so the HMI can display it.

Fail-safe: startup defaults to MANUAL with all outputs LOW, and switching INTO
MANUAL re-zeroes the manual latches, so nothing moves until an operator commands it.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class CellMode(Node):
    def __init__(self):
        super().__init__("cell_mode")

        start_mode = self.declare_parameter("start_mode", "manual").value
        self.auto_mode = str(start_mode).lower() == "auto"

        # Which MANUAL implementation is live (dual structure for debugging):
        #   "plc" -> manual runs on the PLC (FB_CellManual); the effective command
        #            arrives on /plc/* just like AUTO, so we always pass /plc/*.
        #   "ros" -> manual is arbitrated here from the GUI's /manual/* topics,
        #            bypassing the PLC (handy for debugging the ROS path).
        # Default "ros": MANUAL is arbitrated here from /manual/* (fail-safe — the
        # cell stays put until an operator commands). Use "plc" only once the PLC
        # main is actually gated by auto_mode (FB_CellManual), else the cell would
        # follow the PLC's ungated output even in MANUAL.
        self.manual_source = str(
            self.declare_parameter("manual_source", "ros").value
        ).lower()

        self._auto = {"belt": False, "start": False}
        self._manual = {"belt": False, "start": False}

        # AUTO source (from plc_bridge, unchanged topic names)
        self.create_subscription(Bool, "/plc/belt_run",
                                 lambda m: self._set("_auto", "belt", m), 10)
        self.create_subscription(Bool, "/plc/robot_start",
                                 lambda m: self._set("_auto", "start", m), 10)
        # MANUAL source (from the GUI bridge)
        self.create_subscription(Bool, "/manual/belt_run",
                                 lambda m: self._set("_manual", "belt", m), 10)
        self.create_subscription(Bool, "/manual/robot_start",
                                 lambda m: self._set("_manual", "start", m), 10)

        # Arbitrated output -> cell_io, plus current mode -> GUI
        self.pub_belt = self.create_publisher(Bool, "/cmd/belt_run", 10)
        self.pub_start = self.create_publisher(Bool, "/cmd/robot_start", 10)
        self.pub_mode = self.create_publisher(Bool, "/cell/mode", 10)
        # Tell the PLC the current mode so it gates its own sequence.
        # plc_bridge writes this onto the PLC's (writable) auto_mode variable.
        self.pub_plc_mode = self.create_publisher(Bool, "/plc/write/auto_mode", 10)

        self.create_service(SetBool, "/cell/set_mode", self._on_set_mode)

        # Republish at 10 Hz so late-joining subscribers always get current state.
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"cell_mode up (start mode: {'AUTO' if self.auto_mode else 'MANUAL'})"
        )

    def _set(self, which, key, msg):
        getattr(self, which)[key] = bool(msg.data)

    def _on_set_mode(self, req, resp):
        self.auto_mode = bool(req.data)
        if not self.auto_mode:
            # Entering MANUAL: fail-safe, drop all manual latches.
            self._manual = {"belt": False, "start": False}
        mode = "AUTO" if self.auto_mode else "MANUAL"
        self.get_logger().info(f"Mode -> {mode}")
        resp.success = True
        resp.message = mode
        return resp

    def _tick(self):
        if self.auto_mode or self.manual_source != "ros":
            src = self._auto   # /plc/* carries AUTO seq or the PLC manual block
        else:
            src = self._manual  # ROS-side manual path (debug), bypasses the PLC
        self.pub_belt.publish(Bool(data=src["belt"]))
        self.pub_start.publish(Bool(data=src["start"]))
        self.pub_mode.publish(Bool(data=self.auto_mode))
        self.pub_plc_mode.publish(Bool(data=self.auto_mode))


def main():
    rclpy.init()
    node = CellMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
