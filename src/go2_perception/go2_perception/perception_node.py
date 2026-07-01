#!/usr/bin/env python3
"""
perception_node.py
==================
Obstacle proximity detection for the Go2 Resilient Inspector.

Uses /odom/ground_truth to determine robot position and compares it against
the known obstacle positions in inspection_world.world. Publishes a detection
event whenever the robot enters within DETECTION_RANGE of any obstacle, and
a cleared event when it leaves.

Topics
------
Subscribed:
  /odom/ground_truth  (nav_msgs/Odometry)

Published:
  /obstacle_detections  (std_msgs/String)  JSON blob per detection event
  /obstacle_alert       (std_msgs/Bool)    True while any obstacle is in range
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


# Must match obstacle positions in inspection_world.world
OBSTACLES = [
    ("red_box",        2.0, -0.7),
    ("blue_cylinder",  3.7,  1.5),
    ("orange_box",     1.5,  3.7),
    ("green_cylinder", -1.5, 3.7),
    ("yellow_box",    -3.7,  1.5),
]

DETECTION_RANGE = 2.0   # metres


class PerceptionNode(Node):

    def __init__(self):
        super().__init__("perception_node")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._odom_sub = self.create_subscription(
            Odometry, "/odom/ground_truth",
            self._odom_callback, sensor_qos,
        )

        self._det_pub   = self.create_publisher(String, "/obstacle_detections", 10)
        self._alert_pub = self.create_publisher(Bool,   "/obstacle_alert",      10)

        self._robot_x    = 0.0
        self._robot_y    = 0.0
        self._odom_valid = False
        self._in_range: set = set()   # names of obstacles currently in range

        self.create_timer(0.2, self._check_proximity)

        self.get_logger().info(
            f"perception_node ready. Tracking {len(OBSTACLES)} obstacles, "
            f"detection range = {DETECTION_RANGE}m."
        )

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        self._robot_x    = msg.pose.pose.position.x
        self._robot_y    = msg.pose.pose.position.y
        self._odom_valid = True

    # ── Proximity check (5 Hz) ────────────────────────────────────────────────

    def _check_proximity(self) -> None:
        if not self._odom_valid:
            return

        now_in_range: set = set()

        for name, ox, oy in OBSTACLES:
            dist = math.hypot(self._robot_x - ox, self._robot_y - oy)
            if dist <= DETECTION_RANGE:
                now_in_range.add(name)
                if name not in self._in_range:
                    self._on_enter(name, ox, oy, dist)

        for name in self._in_range - now_in_range:
            self.get_logger().info(f"[OBSTACLE CLEARED] {name}")

        self._in_range = now_in_range

        alert      = Bool()
        alert.data = bool(now_in_range)
        self._alert_pub.publish(alert)

    def _on_enter(self, name: str, ox: float, oy: float, dist: float) -> None:
        self.get_logger().warn(
            f"[OBSTACLE DETECTED] {name}  dist={dist:.2f}m  "
            f"robot=({self._robot_x:.2f}, {self._robot_y:.2f})  "
            f"obstacle=({ox}, {oy})"
        )
        payload = String()
        payload.data = json.dumps({
            "obstacle":   name,
            "distance_m": round(dist, 3),
            "robot_x":    round(self._robot_x, 3),
            "robot_y":    round(self._robot_y, 3),
        })
        self._det_pub.publish(payload)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
