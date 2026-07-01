# go2-resilient-inspector
> A quadruped robot is deployed alone into an offshore facility no human can safely enter. It autonomously inspects machinery by reading live health telemetry against real thresholds. Partway through the mission, its LiDAR fails. Instead of aborting, it detects the failure, degrades its navigation confidence and speed, and completes the mission — then hands back a structured report that honestly flags which readings happened under reduced confidence.

**Platform:** Unitree Go2 (simulated) | **Stack:** ROS2 Humble + Gazebo Classic 11 + BehaviorTree.CPP v4 + champ quadruped framework

---

## Demo
[![Go2 Resilient Inspector Demo](https://img.youtube.com/vi/PLACEHOLDER/maxresdefault.jpg)](https://www.youtube.com/watch?v=PLACEHOLDER)

*Watch the lidar fault get injected mid-patrol, the fault detection and mode switch in the terminal, and the robot completing the remaining stations at reduced speed and confidence.*

---

## System Architecture

┌──────────────────────────────────────────────────────────────────┐
│                      go2-resilient-inspector                     │
│                                                                  │
│  ┌──────────────┐   ┌───────────────┐   ┌──────────────────┐    │
│  │ go2_hal_node │   │ sim_machine_  │   │ lidar_sim_node   │    │
│  │  (C++ HAL,   │   │ emulator      │   │ (wall-aware      │    │
│  │  mock mode)  │   │ (CAN telemetry│   │  /go2/scan,      │    │
│  └──────┬───────┘   │  per station) │   │  fault-injectable)│    │
│         │            └──────┬────────┘   └────────┬─────────┘   │
│         │ joint_states       │ can_telemetry        │ /go2/scan  │
│         │                    │                      │            │
│  ┌──────▼───────┐            │            ┌─────────▼─────────┐  │
│  │ champ_bringup│            │            │ topic_health_     │  │
│  │ (locomotion, │            │            │ monitor           │  │
│  │  its own EKFs│            │            │ (independent      │  │
│  │  for gait)   │            │            │  watchdog)        │  │
│  └──────┬───────┘            │            └────────┬─────────┘  │
│         │ /odom/ground_truth │                      │ FaultEvent │
│         │                    │                      │            │
│  ┌──────▼────────────────────▼──────────────────────▼─────────┐ │
│  │                     mission_bt_node                        │ │
│  │   NavigateToStation (proportional controller on ground-    │ │
│  │   truth odom) + InspectStation (real CAN threshold checks) │ │
│  │   + fault handler (speed cap + confidence drop on fault)   │ │
│  └──────────────────────────────┬─────────────────────────────┘ │
│                                  │ inspection_log                │
│                        ┌─────────▼─────────┐                    │
│                        │ inspection_report_ │                    │
│                        │ gen (JSON + MD)    │                    │
│                        └────────────────────┘                    │
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐                            │
│  │ ekf_filter_  │   │ perception_  │                            │
│  │ node         │   │ node         │                            │
│  │ (fixed, runs,│   │ (lookup-table│                            │
│  │  NOT yet     │   │  obstacle    │                            │
│  │  consumed by │   │  proximity,  │                            │
│  │  navigation) │   │  no real     │                            │
│  │              │   │  vision yet) │                            │
│  └──────────────┘   └──────────────┘                            │
└───────────────────────────────────────────────────────────────────┘

### Node summary
| Node | Language | Role |
|---|---|---|
| `go2_hal_node` | C++ | Hardware abstraction — mocks the real Go2 SDK's UDP interface |
| `champ_bringup` | (champ framework) | Quadruped locomotion/gait control; publishes `/odom/ground_truth` |
| `sim_machine_emulator` | Python | Publishes real per-station CAN telemetry (2 Hz), seeded anomalies at station_2 and station_4 |
| `lidar_sim_node` | Python | Ray-cast-aware `/go2/scan` at 10 Hz; supports fault injection via `/fault_inject/lidar` |
| `topic_health_monitor` | Python | Independent watchdog; detects topic silence, publishes `FaultEvent` — runs as a separate process so a planner crash can't disable it |
| `mission_bt_node` | C++ | BehaviorTree.CPP v4 patrol logic. Navigates using a proportional controller on `/odom/ground_truth`. On `FaultEvent`, caps speed (0.30 → 0.15 m/s) and drops confidence (0.95 → 0.60) for all subsequent readings |
| `inspection_report_gen` | Python | Aggregates results into JSON + Markdown reports |
| `ekf_filter_node` | C++ (robot_localization) | Fuses `/go2/odom` + `/go2/imu`. Currently running correctly but **not yet consumed by navigation** — reserved for future Nav2 integration |
| `perception_node` | Python | Lookup-table-based obstacle proximity detection against a fixed set of known coordinates — a placeholder for real sensor-based perception, not image classification |
| `joy_teleop_node` | C++ | PS5 controller manual override — written, not yet wired into the launch file |

### Custom message types (`go2_interfaces`)
- `CanFrame.msg` — per-station CAN telemetry (temp, vibration, error code)
- `FaultEvent.msg` — fault detection notification (subsystem, type, recovery action)
- `InspectionResult.msg` — per-station inspection record with a confidence field

---

## The Fault Tolerance Mechanism

The differentiating feature of this system is what happens when the LiDAR is killed mid-mission.

1. `lidar_sim_node`'s `/go2/scan` goes silent (fault injected via `scripts/inject_fault.sh lidar`)
2. `topic_health_monitor` detects the silence within **2 seconds** and publishes a `FaultEvent` on `/mission/fault_event`
3. `mission_bt_node` receives the event and sets a global `degraded_mode = true` flag
4. Max velocity drops from 0.30 m/s to 0.15 m/s
5. All subsequent `InspectionResult` messages carry `confidence = 0.60` instead of `0.95`
6. The mission completes — it does not abort

**Honest disclosure:** navigation in this version reads `/odom/ground_truth` directly (Gazebo's simulated ground-truth position), not a sensor-fused estimate. The LiDAR fault degrades speed and confidence, but it does not currently force a switch between two different navigation algorithms, because ground-truth odometry is used in both modes. The `ekf_filter_node` (robot_localization) is implemented, fixed, and runs cleanly, but its fused output isn't wired into navigation yet — that's the next milestone, alongside Nav2 integration. The confidence drop still faithfully reflects real degraded sensor coverage and is reported honestly in the final output.

---

## Build & Run

### Prerequisites
```bash
# Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11
sudo apt install ros-humble-desktop ros-humble-robot-localization \
  ros-humble-behaviortree-cpp-v4 ros-humble-joy
```

### Build
```bash
cd Go2-Resilient-Inspector
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
A handful of vendored third-party robot-config packages (other quadrupeds supported by the champ framework) are marked `COLCON_IGNORE` since they're ROS1/catkin and not needed here.

### Run: Happy path (no fault)
```bash
ros2 launch go2_bringup inspection.launch.py
```
Patrols all 5 stations at normal speed and confidence (0.95), generates a report at mission end.

### Run: With fault injection
In a second terminal, once the mission is underway:
```bash
./scripts/inject_fault.sh lidar
```
Watch the first terminal for the fault detection and mode switch. Restore with:
```bash
./scripts/restore_sensors.sh lidar
```

### Monitor live
```bash
ros2 topic echo /mission/fault_event
ros2 topic echo /mission/inspection_log
```

### View BT live in Groot2
```bash
ros2 launch go2_bringup inspection.launch.py enable_groot2:=true
```

### View generated report
```bash
cat /tmp/go2_inspection_reports/inspection_report_*.md
```

---

## Verification Checklist
Confirmed via live test runs:
- [x] Health monitor flags a killed topic within 2 seconds
- [x] BT switches to degraded mode within the same tick as receiving `FaultEvent`
- [x] Happy-path mission completes cleanly (5/5 stations, confidence 0.95, zero fault events)
- [x] Fault-injection mission completes cleanly (mid-mission NORMAL → DEGRADED transition, correct report)
- [x] Station_2 and station_4 correctly flagged as anomalous; stations 1, 3, 5 clean
- [x] Report Markdown and JSON generated correctly and are human-readable

Not yet verified:
- [ ] EKF localization accuracy against ground truth (EKF is fixed and running, but not yet consumed by navigation or benchmarked)
- [ ] Repeated fault-injection runs (only tested once per scenario so far, not a statistical sample)

---

## Code Reuse & Attribution
This project synthesizes patterns from prior work:

| Prior project | What was reused |
|---|---|
| `ROS2-system-inspector` | Heartbeat watchdog pattern in `topic_health_monitor` |
| `can-fault-monitor` | CAN message structure and fault taxonomy |
| `plc-kalman-ekf` | Structured telemetry pattern, EKF covariance tuning approach |

Reusing and citing these is intentional: this project demonstrates system integration, not isolated component novelty.

---

## Design Documents
- [HAL_DESIGN.md](HAL_DESIGN.md) — Real-vs-mock SDK boundary specification. Read this before asking "would this work on real hardware?"

---

## What this project is NOT
- Not a real offshore deployment (see HAL_DESIGN.md for the mock boundary)
- Not a multi-robot system
- Not using Isaac Sim (Gazebo Classic 11 only — confirmed design decision, chosen for VM compatibility)
- Not using real CAN/Modbus hardware (simulated topic publishing)
- Not using real computer vision — `perception_node` is a lookup-table placeholder for a future real perception integration
- Not using Nav2 yet — navigation is a simple proportional controller on ground-truth odometry; Nav2 + EKF-fused localization is the next milestone

---

## Author
Asrith Pandreka — [asrithmoose148@gmail.com](mailto:asrithmoose148@gmail.com)