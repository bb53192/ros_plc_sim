#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""color_classifier: classify the color of the workpiece under one overhead station camera.

Subscribes to a station camera image (sensor_msgs/Image, bridged from the gz camera) and
publishes the dominant color class as std_msgs/String on <output_topic>:
  "blue", "green", "other" (red/yellow/anything else), or "none" (no saturated object in view).

Robust to belt/background: the belt is neutral gray (low saturation), so we only look at
SATURATED pixels in a center ROI and take their mean hue. Launched once per station:

  ros2 run ros_plc_sim color_classifier.py --ros-args \
    -p image_topic:=/station_b/image -p output_topic:=/station_b/part_color
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# OpenCV HSV hue is 0..179. Bands for the two sorted colors.
GREEN_LO, GREEN_HI = 35, 85
BLUE_LO, BLUE_HI = 95, 135


class ColorClassifier(Node):
    def __init__(self) -> None:
        super().__init__("color_classifier")
        self.declare_parameter("image_topic", "/station_a/image")
        self.declare_parameter("output_topic", "/station_a/part_color")
        self.declare_parameter("roi_frac", 0.55)     # center ROI fraction (wide enough for an off-center part)
        self.declare_parameter("sat_min", 80)        # min saturation to count as "colored"
        self.declare_parameter("val_min", 50)        # min value (brightness)
        self.declare_parameter("min_pixels", 12)     # need at least this many colored pixels (low-res cameras)
        self.image_topic = self.get_parameter("image_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.roi_frac = float(self.get_parameter("roi_frac").value)
        self.sat_min = int(self.get_parameter("sat_min").value)
        self.val_min = int(self.get_parameter("val_min").value)
        self.min_pixels = int(self.get_parameter("min_pixels").value)

        self.bridge = CvBridge()
        self._last = None
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.create_timer(0.2, self._tick)  # publish at 5 Hz
        self.get_logger().info(
            f"color_classifier: {self.image_topic} -> {self.output_topic}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self._last = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failed: {exc}")

    def _classify(self, bgr) -> str:
        h, w = bgr.shape[:2]
        rh, rw = int(h * self.roi_frac), int(w * self.roi_frac)
        cy, cx = h // 2, w // 2
        roi = bgr[cy - rh // 2: cy + rh // 2, cx - rw // 2: cx + rw // 2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        mask = (sat >= self.sat_min) & (val >= self.val_min)
        n = int(mask.sum())
        if n < self.min_pixels:
            return "none"
        mean_hue = float(hue[mask].mean())
        if GREEN_LO <= mean_hue <= GREEN_HI:
            return "green"
        if BLUE_LO <= mean_hue <= BLUE_HI:
            return "blue"
        return "other"

    def _tick(self) -> None:
        if self._last is None:
            return
        self.pub.publish(String(data=self._classify(self._last)))


def main() -> None:
    rclpy.init()
    node = ColorClassifier()
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
