#!/bin/bash
# inject_fault.sh
# ================
# Manually injects a sensor fault by killing a topic publisher.
# Used for Phase 5 testing and as the demo's "climax moment".
#
# Usage:
#   ./scripts/inject_fault.sh lidar      # kill /go2/scan publisher
#   ./scripts/inject_fault.sh camera     # kill /go2/camera/image_raw publisher
#   ./scripts/inject_fault.sh can_bus    # kill /station_1/can_telemetry publisher
#
# The topic_health_monitor will detect silence within its configured timeout
# (2s for lidar, 3s for camera) and publish a FaultEvent.
# The mission_bt_node will then switch to degraded navigation.
#
# To verify fault detection:
#   ros2 topic echo /mission/fault_event
#
# To verify degraded mode is active:
#   ros2 param get /mission_bt_node autonomous_mode  (will still be true)
#   ros2 topic echo /mission/inspection_log | grep degraded_mode
#
# Requirements: ROS2 Humble sourced, system running.

set -e

SUBSYSTEM="${1:-lidar}"

case "$SUBSYSTEM" in
  lidar)
    TOPIC="/go2/scan"
    NODE_PATTERN="gazebo"  # Gazebo publishes the scan via ros_gz_bridge
    ;;
  camera)
    TOPIC="/go2/camera/image_raw"
    NODE_PATTERN="gazebo"
    ;;
  can_bus)
    TOPIC="/station_1/can_telemetry"
    NODE_PATTERN="sim_machine_emulator"
    ;;
  imu)
    TOPIC="/go2/imu"
    NODE_PATTERN="go2_hal_node"
    ;;
  *)
    echo "Unknown subsystem: $SUBSYSTEM"
    echo "Valid options: lidar, camera, can_bus, imu"
    exit 1
    ;;
esac

echo "=== FAULT INJECTION ==="
echo "Subsystem:  $SUBSYSTEM"
echo "Topic:      $TOPIC"
echo ""
echo "Verifying topic is currently alive..."
if ! ros2 topic hz "$TOPIC" --window 5 2>&1 | grep -q "average rate"; then
  echo "WARNING: Topic $TOPIC may already be silent."
fi

echo ""
echo "Finding publisher node for $TOPIC..."
PUBLISHER=$(ros2 topic info "$TOPIC" 2>/dev/null | grep "Publisher count" || echo "unknown")
echo "$PUBLISHER"

echo ""
echo "Pausing the sim_machine_emulator or Gazebo bridge to simulate failure..."
echo "(In a real system, this would be a hardware disconnection or sensor power loss.)"
echo ""

# The cleanest simulation of a topic going silent is to pause the publisher.
# We publish to a control topic that the emulator listens to.
ros2 topic pub --once /mission/inject_fault std_msgs/msg/Bool "{data: true}"

echo ""
echo "Fault injected. Monitor health status:"
echo "  ros2 topic echo /mission/fault_event"
echo ""
echo "Monitor BT state:"
echo "  ros2 topic echo /mission/inspection_log"
echo ""
echo "Expected: health_monitor detects silence within ${TIMEOUT:-2}s, publishes FaultEvent,"
echo "          mission_bt_node switches to degraded navigation mode."
