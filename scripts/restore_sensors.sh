#!/usr/bin/env bash
# restore_sensors.sh
# ===================
# Restore a previously injected sensor fault.
#
# Usage:
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
    echo "[restore_sensors] Clearing LiDAR fault..."
    ros2 topic pub --once /fault_inject/lidar std_msgs/msg/Bool "data: false"
    echo "[restore_sensors] Done. /go2/scan will resume publishing within 100ms."
    ;;
  *)
    echo "Unknown subsystem: $SUBSYSTEM"
    echo "Available: lidar"
    exit 1
    ;;
esac
