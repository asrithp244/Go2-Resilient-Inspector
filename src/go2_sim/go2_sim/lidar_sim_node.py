#!/usr/bin/env python3
"""
lidar_sim_node.py
=================
Simulated 2D LiDAR for the Go2 inspection robot.

Computes ray-cast distances against the inspection room's known box geometry
so the published scan reflects the real simulation environment. Publishes
sensor_msgs/LaserScan on /go2/scan at 10 Hz from frame go2/lidar_link.

In a real deployment, replace this node with the physical LiDAR ROS2 driver
publishing on the same topic and QoS. The health monitor and all downstream
consumers are source-agnostic — they only care that /go2/scan publishes.

Fault injection
---------------
Publish True to /fault_inject/lidar to simulate LiDAR going silent:
    ros2 topic pub --once /fault_inject/lidar std_msgs/msg/Bool "data: true"

Or use the helper script:
    ./scripts/inject_fault.sh lidar

The topic_health_monitor detects silence within 2s and publishes a FaultEvent
on /mission/fault_event, which mission_bt_node uses to switch to degraded mode.

To restore:
    ros2 topic pub --once /fault_inject/lidar std_msgs/msg/Bool "data: false"
    ./scripts/restore_sensors.sh lidar
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


# Room geometry — must match inspection_world.world and room model.sdf
ROOM_HALF_X = 2.2   # east / west walls at x = ±2.2 m
ROOM_HALF_Y = 2.0   # north / south walls at y = ±2.0 m

# LiDAR parameters
NUM_RAYS        = 360
ANGLE_MIN       = -math.pi
ANGLE_MAX       =  math.pi
RANGE_MIN       = 0.10    # metres
RANGE_MAX       = 10.0    # metres
PUBLISH_HZ      = 10.0
LIDAR_FRAME_ID  = "go2/lidar_link"

# Gaussian noise added to each ray to simulate a real sensor (std dev in metres)
RANGE_NOISE_STD = 0.02


class LidarSimNode(Node):
    """
    Publishes simulated LaserScan data by ray-casting against the known room walls.

    The scan accurately reflects the robot's line-of-sight distances to all four
    walls based on its current pose from /odom/ground_truth. This is not a lookup
    table — each scan is computed from the robot's live position.
    """

    def __init__(self):
        super().__init__("lidar_sim_node")

        self.declare_parameter("publish_hz",       PUBLISH_HZ)
        self.declare_parameter("num_rays",         NUM_RAYS)
        self.declare_parameter("range_max",        RANGE_MAX)
        self.declare_parameter("noise_enabled",    True)

        hz         = self.get_parameter("publish_hz").value
        self._num_rays    = self.get_parameter("num_rays").value
        self._range_max   = self.get_parameter("range_max").value
        self._noise_on    = self.get_parameter("noise_enabled").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._scan_pub = self.create_publisher(LaserScan, "/go2/scan", sensor_qos)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom/ground_truth",
            self._odom_callback, sensor_qos,
        )

        self._fault_sub = self.create_subscription(
            Bool, "/fault_inject/lidar",
            self._fault_callback, reliable_qos,
        )

        self._robot_x    = 0.0
        self._robot_y    = 0.0
        self._robot_yaw  = 0.0
        self._odom_valid = False
        self._faulted    = False

        import random
        self._rng = random.Random(42)

        self.create_timer(1.0 / hz, self._publish_scan)

        self.get_logger().info(
            f"lidar_sim_node started. Publishing /go2/scan at {hz:.0f} Hz "
            f"({self._num_rays} rays, range_max={self._range_max:.1f}m). "
            "Inject fault: ros2 topic pub --once /fault_inject/lidar "
            "std_msgs/msg/Bool \"data: true\""
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_x = p.x
        self._robot_y = p.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._odom_valid = True

    def _fault_callback(self, msg: Bool) -> None:
        if msg.data and not self._faulted:
            self._faulted = True
            self.get_logger().warn(
                "FAULT INJECTED: /go2/scan publication stopped. "
                "topic_health_monitor will detect silence within 2s."
            )
        elif not msg.data and self._faulted:
            self._faulted = False
            self.get_logger().info("Fault cleared: /go2/scan publication resumed.")

    # ── Scan publication ───────────────────────────────────────────────────────

    def _publish_scan(self) -> None:
        if self._faulted:
            return   # silent — health monitor detects this

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = LIDAR_FRAME_ID
        msg.angle_min       = ANGLE_MIN
        msg.angle_max       = ANGLE_MAX
        msg.angle_increment = (ANGLE_MAX - ANGLE_MIN) / self._num_rays
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / PUBLISH_HZ
        msg.range_min       = RANGE_MIN
        msg.range_max       = self._range_max
        msg.ranges          = self._compute_ranges()
        msg.intensities     = [100.0] * self._num_rays
        self._scan_pub.publish(msg)

    def _compute_ranges(self) -> list:
        """
        Ray-cast against the four room walls from the robot's current position.

        For each ray angle, find the nearest wall intersection and return
        that distance (clamped to range_max). Gaussian noise is added to
        simulate a real sensor's measurement uncertainty.
        """
        ranges = []
        angle_inc = (ANGLE_MAX - ANGLE_MIN) / self._num_rays
        rx, ry, yaw = self._robot_x, self._robot_y, self._robot_yaw

        for i in range(self._num_rays):
            alpha = ANGLE_MIN + i * angle_inc + yaw
            cos_a = math.cos(alpha)
            sin_a = math.sin(alpha)
            best_t = self._range_max

            # East wall  x = +ROOM_HALF_X
            if abs(cos_a) > 1e-9:
                t = (ROOM_HALF_X - rx) / cos_a
                if RANGE_MIN < t < best_t:
                    y_hit = ry + t * sin_a
                    if abs(y_hit) <= ROOM_HALF_Y:
                        best_t = t

            # West wall  x = -ROOM_HALF_X
            if abs(cos_a) > 1e-9:
                t = (-ROOM_HALF_X - rx) / cos_a
                if RANGE_MIN < t < best_t:
                    y_hit = ry + t * sin_a
                    if abs(y_hit) <= ROOM_HALF_Y:
                        best_t = t

            # North wall  y = +ROOM_HALF_Y
            if abs(sin_a) > 1e-9:
                t = (ROOM_HALF_Y - ry) / sin_a
                if RANGE_MIN < t < best_t:
                    x_hit = rx + t * cos_a
                    if abs(x_hit) <= ROOM_HALF_X:
                        best_t = t

            # South wall  y = -ROOM_HALF_Y
            if abs(sin_a) > 1e-9:
                t = (-ROOM_HALF_Y - ry) / sin_a
                if RANGE_MIN < t < best_t:
                    x_hit = rx + t * cos_a
                    if abs(x_hit) <= ROOM_HALF_X:
                        best_t = t

            # Add sensor noise
            if self._noise_on and best_t < self._range_max:
                best_t += self._rng.gauss(0.0, RANGE_NOISE_STD)
                best_t = max(RANGE_MIN, min(best_t, self._range_max))

            ranges.append(float(best_t))

        return ranges


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
