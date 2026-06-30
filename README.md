# go2-resilient-inspector

> A quadruped robot is deployed alone into an offshore facility no human can safely enter. It autonomously inspects machinery by reading live health telemetry and visually verifying status. Partway through the mission, its primary navigation sensor fails. Instead of aborting, it detects the failure, switches to a degraded navigation mode, and completes the mission — then hands back a structured report that honestly flags which readings happened under reduced confidence.

**Platform:** Unitree Go2 (simulated) | **Stack:** ROS2 Humble + Gazebo Harmonic + BehaviorTree.CPP v4 + Nav2

---

## Demo

[![Go2 Resilient Inspector Demo](https://img.youtube.com/vi/PLACEHOLDER/maxresdefault.jpg)](https://www.youtube.com/watch?v=PLACEHOLDER)

*4:30 — Watch the lidar fault injected at 2:30, the BT switch in terminal, and the robot completing the remaining stations at reduced speed.*

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      go2-resilient-inspector                    │
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ go2_hal_node │    │ go2_sim      │    │ go2_perception   │  │
│  │   (C++ HAL)  │    │ (CAN emitter)│    │  (YOLOv8/HSV)   │  │
│  └──────┬───────┘    └──────┬───────┘    └────────┬─────────┘  │
│         │ joint_states       │ can_telemetry        │ anomaly    │
│         │ imu, odom          │                      │ detected   │
│  ┌──────▼───────┐            │            ┌─────────▼─────────┐  │
│  │ ekf_filter   │            │            │  mission_bt_node  │  │
│  │ (odom + IMU) │            └────────────►  (BT.CPP v4)      │  │
│  └──────┬───────┘                         │  waypoint patrol  │  │
│         │ /tf odom→base_link              │  + fault response │  │
│         │                      FaultEvent │                   │  │
│  ┌──────▼────────────────────────────────►│                   │  │
│  │ topic_health_monitor  ◄──── /go2/scan  └─────────┬─────────┘  │
│  │ (independent watchdog)     /go2/camera            │            │
│  └───────────────────────────────────────    inspection_log │    │
│                                                             │    │
│  ┌──────────────────────────────────────────┐  ┌──────────▼───┐  │
│  │           Nav2 Stack                     │  │ report_gen   │  │
│  │  global costmap → NavFn → DWB local      │  │ JSON + MD    │  │
│  └──────────────────────────────────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Node summary

| Node | Language | Role |
|---|---|---|
| `go2_hal_node` | C++ | Hardware abstraction — mocks go2_ros2_sdk UDP interface |
| `ekf_filter_node` | C++ (robot_localization) | Fuses odom + IMU → stable pose |
| `topic_health_monitor` | Python | Heartbeat watchdog; publishes `FaultEvent` on silence |
| `sim_machine_emulator` | Python | Fake CAN/Modbus telemetry per inspection station |
| `perception_node` | Python | HSV/YOLOv8 visual anomaly + telemetry threshold checks |
| `mission_bt_node` | C++ | BT.CPP v4 patrol + fault-triggered degraded navigation |
| `inspection_report_gen` | Python | Aggregates results → JSON + Markdown report |
| `joy_teleop_node` | C++ | PS5 controller, toggle autonomous/manual |

### Custom message types (`go2_interfaces`)

- `CanFrame.msg` — per-station CAN telemetry (temp, vibration, error code)
- `FaultEvent.msg` — fault detection notification (subsystem, type, recovery action)
- `InspectionResult.msg` — per-station inspection record with confidence field

---

## The Fault Tolerance Mechanism

The differentiating feature of this system is what happens at 2:30 in the demo.

1. The lidar topic (`/go2/scan`) goes silent — simulating a sensor failure
2. `topic_health_monitor` detects silence within **2 seconds** and publishes a `FaultEvent`
3. `mission_bt_node` receives the event and sets a global `degraded_mode = true` flag
4. Subsequent `NavigateToStation` actions switch from Nav2 (lidar-dependent) to dead-reckoning (odom + IMU only)
5. Max velocity drops from 0.6 m/s to 0.3 m/s
6. All `InspectionResult` messages after the fault carry `confidence = 0.60` instead of `0.95`
7. The mission completes — it does not abort

The confidence reduction is not just a flag: it appears in the final report, explicitly marking which readings are less certain. This is what honest engineering looks like.

---

## Build & Run

### Prerequisites

```bash
# Ubuntu 22.04 + ROS2 Humble + Gazebo Harmonic
sudo apt install ros-humble-desktop ros-humble-nav2-bringup \
  ros-humble-robot-localization ros-humble-behaviortree-cpp \
  ros-humble-joy ros-humble-cv-bridge python3-opencv

# BehaviorTree.CPP v4 (if not in apt)
# See: https://www.behaviortree.dev/
```

### Build

```bash
cd /path/to/go2-resilient-inspector
colcon build --symlink-install --packages-select \
  go2_interfaces go2_hal go2_localization go2_health_monitor \
  go2_mission go2_sim go2_perception go2_report go2_teleop go2_bringup
source install/setup.bash
```

Build order matters: `go2_interfaces` must build first (custom messages).

### Run: Happy path (no fault)

```bash
ros2 launch go2_bringup inspection_mission.launch.py
```

### Run: With fault injection at 60 seconds

```bash
ros2 launch go2_bringup inspection_mission.launch.py \
  inject_fault:=true \
  fault_delay_sec:=60.0
```

### Run: Manual fault injection (demo use)

```bash
# In a separate terminal, at the moment you want for the demo:
./scripts/inject_fault.sh lidar
```

### Monitor fault detection

```bash
ros2 topic echo /mission/fault_event
ros2 topic echo /mission/inspection_log
```

### View BT live in Groot2

```bash
ros2 launch go2_bringup inspection_mission.launch.py enable_groot2:=true
# Then open Groot2 and connect to port 1667
```

### View generated report

Reports are written to `/tmp/go2_inspection_reports/` at mission completion.

```bash
cat /tmp/go2_inspection_reports/inspection_report_*.md
```

Sample outputs are committed in `reports/` — see `inspection_report.md` and `inspection_report.json`.

---

## Verification Checklist

- [ ] EKF localization error under 5cm position, 2 degrees orientation vs Gazebo ground truth
- [ ] Health monitor flags a killed topic within 2 seconds, 10/10 test runs
- [ ] BT switches to degraded navigation within 1 tick of receiving FaultEvent
- [ ] Mission completes without crash in 10/10 fault-injection test runs
- [ ] Station 2 and Station 4 correctly flagged as anomalous; stations 1, 3, 5 clean
- [ ] Report JSON validates against schema; Markdown is human-readable without explanation
- [ ] Full happy-path mission completes via single launch file
- [ ] Full fault-injection mission completes via single launch file with `inject_fault:=true`

---

## Code Reuse & Attribution

This project synthesizes patterns from prior work:

| Prior project | What was reused |
|---|---|
| `bt_nav_demo` | BT.CPP mission XML structure, NavigateToPose action node pattern |
| `ROS2-system-inspector` | Heartbeat watchdog pattern in `topic_health_monitor` |
| `can-fault-monitor` | CAN message structure and fault taxonomy |
| `plc-kalman-ekf` | Structured Modbus-style telemetry, EKF covariance tuning approach |

Reusing and citing these is intentional: this project demonstrates system integration, not isolated component novelty.

---

## Design Documents

- [HAL_DESIGN.md](HAL_DESIGN.md) — Real-vs-mock SDK boundary specification. Read this before asking "would this work on real hardware?"

---

## What this project is NOT

- Not a real offshore deployment (see HAL_DESIGN.md for the mock boundary)
- Not a multi-robot system
- Not using Isaac Sim (Gazebo only — confirmed design decision)
- Not using real CAN/Modbus hardware (simulated topic publishing)
- Not claiming production-ready perception (rule-based classifier sufficient for demo)

---

## Author

Asrith Pandreka — [apandrek@asu.edu](mailto:apandrek@asu.edu)
