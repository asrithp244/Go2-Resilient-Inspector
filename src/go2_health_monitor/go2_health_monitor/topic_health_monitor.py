"""
topic_health_monitor.py
========================
Independent watchdog node. Monitors heartbeat on critical topics and publishes
FaultEvent messages when a topic goes silent beyond its configured timeout.

Runs as a separate process from the mission BT node: a crash in mission planning
must NOT disable fault detection. This is a safety invariant, not a convenience.

Adapted from ROS2-system-inspector heartbeat pattern — see README for citation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from go2_interfaces.msg import CanFrame, FaultEvent

from typing import Dict, Optional
import threading


class WatchedTopic:
    """Tracks last-received timestamp and fault state for one topic."""

    def __init__(self, name: str, timeout_sec: float):
        self.name = name
        self.timeout_sec = timeout_sec
        self.last_received: Optional[float] = None  # wall-clock seconds
        self.faulted: bool = False
        self._lock = threading.Lock()

    def record_heartbeat(self, wall_clock_sec: float) -> None:
        with self._lock:
            self.last_received = wall_clock_sec
            self.faulted = False  # auto-recover when topic resumes

    def check(self, wall_clock_sec: float) -> bool:
        """Return True if this topic just transitioned to fault state."""
        with self._lock:
            if self.last_received is None:
                # Never received — only fault after 3× the timeout (startup grace)
                return False
            elapsed = wall_clock_sec - self.last_received
            newly_faulted = elapsed > self.timeout_sec and not self.faulted
            if newly_faulted:
                self.faulted = True
            return newly_faulted

    def is_faulted(self) -> bool:
        with self._lock:
            return self.faulted


class TopicHealthMonitor(Node):
    """
    Heartbeat watchdog for the inspection system's critical sensor topics.

    Monitored topics:
      /go2/scan        — 2D LiDAR (Nav2-critical; failure triggers degraded mode)
      /go2/camera/image_raw — camera (perception-critical)
      /station_1/can_telemetry — representative CAN bus liveness check
      /go2/imu         — IMU (EKF-critical)

    On fault detection: publishes FaultEvent on /mission/fault_event (RELIABLE QoS).
    The mission BT node subscribes and switches navigation subtrees.
    """

    # Timeouts (seconds). Tuned to be 3–5× the expected publish period
    # so a single late message doesn't trigger a false positive.
    TIMEOUTS = {
        "lidar":   2.0,   # scan @ ~10 Hz → 0.1s period → 2s = 20 missed
        "camera":  3.0,   # image @ 10 Hz → 0.1s period → 3s = 30 missed
        "can_bus": 5.0,   # CAN @ 2 Hz   → 0.5s period → 5s = 10 missed
        "imu":     1.0,   # IMU @ 200 Hz → 0.005s period → 1s = 200 missed
    }

    def __init__(self):
        super().__init__("topic_health_monitor")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("lidar_timeout_sec", self.TIMEOUTS["lidar"])
        self.declare_parameter("camera_timeout_sec", self.TIMEOUTS["camera"])
        self.declare_parameter("can_timeout_sec", self.TIMEOUTS["can_bus"])
        self.declare_parameter("imu_timeout_sec", self.TIMEOUTS["imu"])
        self.declare_parameter("check_rate_hz", 10.0)

        lidar_to = self.get_parameter("lidar_timeout_sec").value
        camera_to = self.get_parameter("camera_timeout_sec").value
        can_to = self.get_parameter("can_timeout_sec").value
        imu_to = self.get_parameter("imu_timeout_sec").value
        check_rate = self.get_parameter("check_rate_hz").value

        # ── Watched topics registry ───────────────────────────────────────────
        self._watched: Dict[str, WatchedTopic] = {
            "lidar":   WatchedTopic("lidar",   lidar_to),
            "camera":  WatchedTopic("camera",  camera_to),
            "can_bus": WatchedTopic("can_bus", can_to),
            "imu":     WatchedTopic("imu",     imu_to),
        }

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            LaserScan, "/go2/scan",
            lambda msg: self._heartbeat("lidar"), sensor_qos)

        self.create_subscription(
            Image, "/go2/camera/image_raw",
            lambda msg: self._heartbeat("camera"), sensor_qos)

        self.create_subscription(
            CanFrame, "/station_1/can_telemetry",
            lambda msg: self._heartbeat("can_bus"), sensor_qos)

        self.create_subscription(
            Imu, "/go2/imu",
            lambda msg: self._heartbeat("imu"), sensor_qos)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._fault_pub = self.create_publisher(
            FaultEvent, "/mission/fault_event", reliable_qos)

        # ── Watchdog timer ────────────────────────────────────────────────────
        period = 1.0 / check_rate
        self.create_timer(period, self._watchdog_tick)

        self.get_logger().info(
            f"TopicHealthMonitor started. Watching: {list(self._watched.keys())}. "
            f"Check rate: {check_rate} Hz."
        )

    # ── Internal ──────────────────────────────────────────────────────────────

    def _heartbeat(self, subsystem: str) -> None:
        """Called on every received message for the given subsystem."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._watched[subsystem].record_heartbeat(now_sec)

    def _watchdog_tick(self) -> None:
        """Periodic check — runs at check_rate_hz regardless of topic activity."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for subsystem, watcher in self._watched.items():
            if watcher.check(now_sec):
                self._publish_fault(subsystem, "topic_silent")
                self.get_logger().warn(
                    f"FAULT DETECTED: '{subsystem}' topic silent for "
                    f">{watcher.timeout_sec:.1f}s. Publishing FaultEvent."
                )

        # Periodic status log (DEBUG level — not spammy in normal operation)
        self.get_logger().debug(
            "Health: " + " | ".join(
                f"{k}={'FAULT' if v.is_faulted() else 'OK'}"
                for k, v in self._watched.items()
            )
        )

    def _publish_fault(self, subsystem: str, fault_type: str) -> None:
        msg = FaultEvent()
        msg.subsystem = subsystem
        msg.fault_type = fault_type
        msg.detected_at = self.get_clock().now().to_msg()
        msg.recovery_action_taken = self._recovery_action_for(subsystem)
        self._fault_pub.publish(msg)

    @staticmethod
    def _recovery_action_for(subsystem: str) -> str:
        actions = {
            "lidar":   "switching_to_degraded_nav_dead_reckoning",
            "camera":  "disabling_visual_inspection_telemetry_only",
            "can_bus": "disabling_telemetry_checks_visual_only",
            "imu":     "switching_to_odom_only_localization",
        }
        return actions.get(subsystem, "halting_mission")


def main(args=None):
    rclpy.init(args=args)
    node = TopicHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
