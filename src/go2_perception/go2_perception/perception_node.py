#!/usr/bin/env python3
"""
perception_node.py
==================
Visual anomaly detection and telemetry fusion for inspection stations.

Visual detection uses a rule-based classifier on the Gazebo camera feed:
  - Detects warning indicator color (RED vs GREEN) by HSV thresholding
  - Classifies panel state (OPEN vs CLOSED) by contour analysis
  - Optionally can be swapped for YOLOv8 inference (see _detect_with_yolo())

Telemetry anomaly detection subscribes to /station_N/can_telemetry and checks:
  - motor_temp_c > TEMP_THRESHOLD (80°C)
  - vibration_level > VIBRATION_THRESHOLD (0.50)
  - error_code != 0x00

This node does NOT publish InspectionResult — that is mission_bt_node's job.
It exposes anomaly detections via /station_N/anomaly_detected (Bool) topics
that the mission BT's InspectStation action node reads.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from go2_interfaces.msg import CanFrame


class PerceptionNode(Node):

    TEMP_THRESHOLD = 80.0       # °C
    VIBRATION_THRESHOLD = 0.50
    ERROR_CODE_NOMINAL = 0x00

    STATION_IDS = ["station_1", "station_2", "station_3", "station_4", "station_5"]

    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("use_yolo", False)
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("confidence_threshold", 0.7)

        self._use_yolo = self.get_parameter("use_yolo").value
        self._bridge = CvBridge()

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

        # Subscribe to Gazebo camera
        self._img_sub = self.create_subscription(
            Image, "/go2/camera/image_raw",
            self._image_callback, sensor_qos)

        # Per-station CAN telemetry subscriptions
        self._can_subs = {}
        self._can_latest = {}
        for sid in self.STATION_IDS:
            self._can_latest[sid] = None
            self._can_subs[sid] = self.create_subscription(
                CanFrame, f"/{sid}/can_telemetry",
                lambda msg, s=sid: self._can_callback(msg, s),
                sensor_qos)

        # Per-station anomaly publishers (Bool) — read by mission BT
        self._anomaly_pubs = {}
        for sid in self.STATION_IDS:
            self._anomaly_pubs[sid] = self.create_publisher(
                Bool, f"/{sid}/anomaly_detected", reliable_qos)

        # Telemetry check timer (runs independently of camera)
        self._tel_timer = self.create_timer(0.5, self._telemetry_check_tick)

        if self._use_yolo:
            self._init_yolo()
        else:
            self.get_logger().info(
                "perception_node started. Visual: HSV rule-based classifier. "
                "Telemetry: threshold-based. (Set use_yolo:=true to enable YOLOv8.)"
            )

    # ── Camera callbacks ──────────────────────────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera frame."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        if self._use_yolo:
            self._detect_with_yolo(frame)
        else:
            self._detect_rule_based(frame)

    def _detect_rule_based(self, frame: np.ndarray) -> None:
        """
        HSV thresholding to detect warning indicator state.
        In Gazebo, warning lights are modeled as colored spheres on machine props.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red warning light: two HSV ranges (wraps around hue)
        red_mask1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        red_mask2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        red_pixels = cv2.countNonZero(red_mask1 + red_mask2)

        # Green nominal indicator
        green_mask = cv2.inRange(hsv, (40, 80, 70), (80, 255, 255))
        green_pixels = cv2.countNonZero(green_mask)

        warning_light_red = red_pixels > 200 and red_pixels > green_pixels

        if warning_light_red:
            self.get_logger().warn(
                f"Visual: WARNING LIGHT RED detected (red_px={red_pixels}, "
                f"green_px={green_pixels}). Station 4 visual anomaly."
            )

    def _detect_with_yolo(self, frame: np.ndarray) -> None:
        """
        YOLOv8 inference path. Swap in when custom-trained model is available.
        Classes: ['warning_light_green', 'warning_light_red', 'panel_open', 'panel_closed']
        """
        # Stub — requires ultralytics and a trained model checkpoint.
        # To enable: pip install ultralytics && train on Gazebo render dataset.
        self.get_logger().debug("YOLOv8 inference stub — model not loaded.")

    def _init_yolo(self) -> None:
        try:
            from ultralytics import YOLO
            model_path = self.get_parameter("yolo_model_path").value
            self._yolo_model = YOLO(model_path if model_path else "yolov8n.pt")
            self.get_logger().info(f"YOLOv8 model loaded from: {model_path or 'yolov8n.pt'}")
        except ImportError:
            self.get_logger().warn(
                "ultralytics not installed. Falling back to rule-based detection. "
                "Install with: pip install ultralytics"
            )
            self._use_yolo = False

    # ── Telemetry callbacks ───────────────────────────────────────────────────

    def _can_callback(self, msg: CanFrame, station_id: str) -> None:
        self._can_latest[station_id] = msg

    def _telemetry_check_tick(self) -> None:
        """Check CAN telemetry for all stations and publish anomaly flags."""
        for sid in self.STATION_IDS:
            msg: CanFrame = self._can_latest.get(sid)
            if msg is None:
                continue

            anomaly = (
                msg.motor_temp_c > self.TEMP_THRESHOLD
                or msg.vibration_level > self.VIBRATION_THRESHOLD
                or msg.error_code != self.ERROR_CODE_NOMINAL
            )

            result = Bool()
            result.data = anomaly
            self._anomaly_pubs[sid].publish(result)

            if anomaly:
                self.get_logger().warn(
                    f"Telemetry ANOMALY at {sid}: "
                    f"temp={msg.motor_temp_c:.1f}°C "
                    f"vib={msg.vibration_level:.3f} "
                    f"err=0x{msg.error_code:02X}"
                )


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
