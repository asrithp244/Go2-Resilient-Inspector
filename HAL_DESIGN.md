# HAL Design: Mock vs. Real Go2 SDK Boundary

## What this document is
This document defines the exact boundary between the **mock hardware abstraction layer** (`go2_hal_node`) used in simulation and the **real Unitree Go2 SDK** it replaces. Any engineer reading this should be able to understand what would need to change to deploy this system on physical hardware. Note: this document is about the hardware/UDP interface boundary specifically. The simulated robot's kinematics use champ's `go1_config`, see the main README for that distinction.

---

## The boundary: one file, one class
Everything in `src/go2_hal/src/go2_hal_node.cpp` is the boundary. Specifically:

| In the mock | In real hardware |
|---|---|
| Subscribes to `/go2/gazebo/joint_states` (Gazebo plugin) | Parses `LowState.motorState[]` from Go2 UDP sport mode socket on `192.168.123.161:8082` |
| Subscribes to `/go2/gazebo/imu` (Gazebo plugin) | Parses `LowState.imu` from same UDP socket |
| Dead-reckoning odometry integration in `publish_odometry()` | Uses `SportModeState.position[]` and `velocity[]` from Go2 sport mode state estimator |
| Foot contact: constant 120N per leg (simulated) | Parses `LowState.footForce[]` (4 x uint16 contact force sensors) |
| Velocity commands: no forwarding needed (Gazebo direct) | Calls `SportModeSDK.Move(vx, vy, vyaw)` via UDP control channel |

The topic names, message types, QoS profiles, and frame IDs published by `go2_hal_node` are **identical in mock and real modes**. All downstream nodes are unaware of which mode is active.

---

## Real SDK reference
The real interface is the `go2_ros2_sdk` (GitHub: unitreerobotics/go2_ros2_sdk).

Key files in the real SDK that `go2_hal_node` replaces:
- `src/go2_ros2_sdk/go2_driver/go2_driver_node.cpp`: the real UDP listener
- `src/go2_ros2_sdk/go2_ros2_sdk/go2_constants.py`: topic name definitions
- `unitree_sdk2/include/unitree/robot/go2/sport/sport_client.hpp`: motion API

The Go2 UDP protocol is documented in Unitree's SDK2 spec. LowState is received at **500 Hz**. The HAL re-publishes joint states at 500 Hz and IMU at 400 Hz, matching the real hardware's output rate. In simulation, these rates are capped by Gazebo's physics step (typically 1 kHz with a 500 Hz plugin publish rate).

---

## Frame tree
Both mock and real modes publish this tf2 tree, used by champ's own gait control:
map
└── odom                    (published by champ's internal EKFs: base_to_footprint_ekf, footprint_to_odom_ekf)
└── base_link           (published by champ_bringup)
├── go2/imu_link   (static: from URDF)
├── go2/lidar_link (static: from URDF)
├── go2/camera_link(static: from URDF)
├── FL_foot        (published by: go2_hal_node foot contact)
├── FR_foot
├── RL_foot
└── RR_foot

No loops. Every frame has exactly one parent. Verified against `ros2 run tf2_tools view_frames`.

Note: this is the physical TF tree champ uses for locomotion. `mission_bt_node`'s own patrol navigation reads `/odom/ground_truth` directly rather than consuming this tree, and the project's own `ekf_filter_node` (robot_localization) runs with `publish_tf: false` so it doesn't touch this transform at all. See the main README for that distinction.

---

## QoS decisions
| Topic | Policy | Rationale |
|---|---|---|
| `/go2/joint_states` | BEST_EFFORT, depth=5 | 500 Hz, fresh data matters more than delivery |
| `/go2/imu` | BEST_EFFORT, depth=5 | 400 Hz, same reasoning |
| `/go2/odom` | RELIABLE, depth=10 | Downstream consumers depend on every message for integration |
| `/cmd_vel` | RELIABLE, depth=10 | Velocity commands must not be silently dropped |
| `/mission/fault_event` | RELIABLE, depth=10 | Safety-critical: fault must be received by BT |
| `/mission/inspection_log` | RELIABLE, depth=10 | Report data must not be lost |

---

## What does NOT change for real hardware
- All downstream node code (health_monitor, mission_bt_node, report_gen)
- Topic names and message types
- QoS profiles
- tf2 frame names
- BehaviorTree.CPP XML mission definition

---

## What DOES change for real hardware
1. Replace `go2_hal_node`'s Gazebo bridge subscribers with `unitree_sdk2` UDP listener
2. Replace `publish_odometry()` dead-reckoning with real sport mode state
3. Replace `publish_foot_contact()` simulation with real `LowState.footForce[]`
4. Replace `cmd_vel_callback()` no-op with `SportClient.Move()` call
5. Add network configuration: Go2 at `192.168.123.161`, laptop at `192.168.123.2`
6. Source `unitree_sdk2` and `go2_ros2_sdk` instead of Gazebo

Real hardware deployment would also need network timing tuning and physical safety testing beyond just swapping these calls, so treat this as a rough scope, not a schedule.

---

## Why mock this way
Mocking at the SDK boundary, not at the sensor level, means:
- The entire software stack above the HAL runs without modification
- Integration tests are meaningful: they test the same code that runs on hardware
- The mock is honest about what it does and doesn't cover

Keeping the hardware interface thin and swappable is a common pattern in field robotics: the software stack above it shouldn't need to know or care whether it's talking to a simulator or a real robot.
