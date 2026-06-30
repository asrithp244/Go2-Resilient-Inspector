"""
sim_machine_emulator.py
========================
Simulates CAN/Modbus telemetry for inspection stations in the offshore facility.

Publishes fake CanFrame messages per station at 2 Hz. Most stations publish
healthy telemetry; stations 2 and 4 are seeded with anomalies.

This node is explicitly a simulation mock. In a real deployment, this would be
replaced by a CAN bus driver reading from physical sensors on each machine.
See README for the real-vs-mock boundary.

Adapted from can-fault-monitor and plc-kalman-ekf telemetry patterns.
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from go2_interfaces.msg import CanFrame


# Station configurations: {station_id: {normal_ranges, anomaly_seeded}}
STATION_CONFIGS = {
    "station_1": {
        "can_id": 0x101,
        "normal_temp_range": (35.0, 55.0),
        "normal_vibration_range": (0.05, 0.25),
        "anomaly_seeded": False,
        "error_code": 0x00,
    },
    "station_2": {
        "can_id": 0x102,
        "normal_temp_range": (40.0, 60.0),
        "normal_vibration_range": (0.60, 0.90),  # HIGH — anomaly
        "anomaly_seeded": True,
        "error_code": 0x03,                       # error code set — anomaly
    },
    "station_3": {
        "can_id": 0x103,
        "normal_temp_range": (30.0, 50.0),
        "normal_vibration_range": (0.05, 0.20),
        "anomaly_seeded": False,
        "error_code": 0x00,
    },
    "station_4": {
        "can_id": 0x104,
        "normal_temp_range": (88.0, 96.0),        # OVERTEMP — anomaly (threshold = 80°C)
        "normal_vibration_range": (0.15, 0.30),
        "anomaly_seeded": True,
        "error_code": 0x00,
    },
    "station_5": {
        "can_id": 0x105,
        "normal_temp_range": (35.0, 55.0),
        "normal_vibration_range": (0.05, 0.25),
        "anomaly_seeded": False,
        "error_code": 0x00,
    },
}


class SimMachineEmulator(Node):
    """
    Publishes simulated CAN telemetry for all inspection stations.

    One instance of this node handles all stations. Each station publishes
    on its own topic: /station_N/can_telemetry at 2 Hz.
    """

    # Anomaly thresholds (documented in config, duplicated here for reference)
    TEMP_ANOMALY_THRESHOLD = 80.0   # °C
    VIBRATION_ANOMALY_THRESHOLD = 0.50

    def __init__(self):
        super().__init__("sim_machine_emulator")

        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("noise_enabled", True)

        rate = self.get_parameter("publish_rate_hz").value
        self._noise = self.get_parameter("noise_enabled").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self._publishers = {}
        for station_id in STATION_CONFIGS:
            topic = f"/{station_id}/can_telemetry"
            self._publishers[station_id] = self.create_publisher(
                CanFrame, topic, sensor_qos)

        self._timer = self.create_timer(1.0 / rate, self._publish_all)
        self._seq = 0

        self.get_logger().info(
            f"SimMachineEmulator: publishing {len(STATION_CONFIGS)} stations at {rate} Hz. "
            f"Anomaly stations: station_2 (vibration+error), station_4 (overtemp)."
        )

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        self._seq += 1

        for station_id, cfg in STATION_CONFIGS.items():
            msg = CanFrame()
            msg.stamp = now
            msg.id = cfg["can_id"]

            # Simulate sensor reading within configured range, with optional noise
            temp = random.uniform(*cfg["normal_temp_range"])
            vibration = random.uniform(*cfg["normal_vibration_range"])
            if self._noise:
                temp += random.gauss(0.0, 0.5)
                vibration += random.gauss(0.0, 0.01)

            msg.motor_temp_c = float(temp)
            msg.vibration_level = max(0.0, float(vibration))
            msg.error_code = cfg["error_code"]

            # Pack raw data bytes (simplified 8-byte CAN frame encoding)
            # Byte 0-1: temp (uint16, 0.1°C resolution)
            # Byte 2-3: vibration (uint16, 0.001 resolution)
            # Byte 4: error_code
            # Byte 5-7: reserved / sequence
            temp_raw = int(temp * 10) & 0xFFFF
            vib_raw = int(vibration * 1000) & 0xFFFF
            msg.data = [
                (temp_raw >> 8) & 0xFF,
                temp_raw & 0xFF,
                (vib_raw >> 8) & 0xFF,
                vib_raw & 0xFF,
                cfg["error_code"],
                (self._seq >> 16) & 0xFF,
                (self._seq >> 8) & 0xFF,
                self._seq & 0xFF,
            ]

            self._publishers[station_id].publish(msg)

            if cfg["anomaly_seeded"]:
                self.get_logger().debug(
                    f"{station_id}: temp={temp:.1f}°C vib={vibration:.3f} "
                    f"err=0x{cfg['error_code']:02X} [ANOMALY]"
                )


def main(args=None):
    rclpy.init(args=args)
    node = SimMachineEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
