#!/usr/bin/env bash
# inject_fault.sh
# ================
# Simulate a sensor failure for the Go2 resilient inspector demo.
#
# Usage:
#   ./scripts/inject_fault.sh lidar
#
# What happens:
#   1. Publishes True to /fault_inject/lidar
#   2. lidar_sim_node stops publishing on /go2/scan (topic goes silent)
#   3. topic_health_monitor detects silence within 2s
#   4. FaultEvent published on /mission/fault_event
#   5. mission_bt_node switches to DEGRADED mode (half speed, confidence=0.60)
#
# To restore normal operation:
#   ./scripts/restore_sensors.sh lidar

set -e

SUBSYSTEM="${1:-}"

if [ -z "$SUBSYSTEM" ]; then
    echo "Usage: $0 <subsystem>"
    echo "Available subsystems: lidar"
    exit 1
fi

case "$SUBSYSTEM" in
  lidar)
    echo "[inject_fault] Injecting LiDAR failure..."
    ros2 topic pub --once /fault_inject/lidar std_msgs/msg/Bool "data: true"
    echo ""
    echo "[inject_fault] Done. /go2/scan is now silent."
    echo "[inject_fault] topic_health_monitor detects silence within 2s."
    echo ""
    echo "Watch for the FaultEvent:"
    echo "    ros2 topic echo /mission/fault_event"
    echo ""
    echo "Watch mission_bt_node switch to DEGRADED mode (check terminal logs)."
    echo ""
    echo "To restore: ./scripts/restore_sensors.sh lidar"
    ;;
  *)
    echo "Unknown subsystem: $SUBSYSTEM"
    echo "Available: lidar"
    exit 1
    ;;
esac
