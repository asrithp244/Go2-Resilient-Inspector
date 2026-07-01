"""
inspection.launch.py
====================
Single launch file for the Go2 Resilient Inspector simulation.

Startup sequence (timed to avoid race conditions):
  t= 0s  Gazebo server + client + robot_state_publisher
  t= 2s  sim_machine_emulator (CAN telemetry for all 5 stations)
  t= 2s  inspection_report_gen (listens for results, writes report at mission end)
  t= 4s  Spawn robot into Gazebo
  t= 6s  go2_hal_node (bridges Gazebo joint/IMU data to Go2 topic namespace)
  t= 9s  champ_bringup (quadruped controller, state estimation, /odom/ground_truth)
  t=11s  EKF (robot_localization, fuses /go2/odom + /go2/imu)
  t=14s  Load joint_states_controller
  t=16s  Load joint_group_effort_controller
  t=17s  lidar_sim_node (ray-cast /go2/scan; kill to inject LiDAR fault)
  t=19s  topic_health_monitor (watchdog, detects /go2/scan silence within 2s)
  t=22s  mission_bt_node (begins patrol once /odom/ground_truth arrives)
  t=25s  perception_node (obstacle proximity detection)

Fault injection (in a separate terminal during the mission):
  ./scripts/inject_fault.sh lidar

Usage:
  ros2 launch go2_bringup inspection.launch.py
  ros2 launch go2_bringup inspection.launch.py enable_groot2:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    enable_groot2 = LaunchConfiguration("enable_groot2", default="false")

    declare_groot2 = DeclareLaunchArgument(
        "enable_groot2",
        default_value="false",
        description="Enable Groot2 BehaviorTree live visualiser on port 1667",
    )

    # ── Package paths ─────────────────────────────────────────────────────────
    go2_bringup_pkg       = get_package_share_directory("go2_bringup")
    champ_description_pkg = get_package_share_directory("champ_description")
    champ_config_pkg      = get_package_share_directory("champ_config")
    champ_bringup_pkg     = get_package_share_directory("champ_bringup")
    go2_localization_pkg  = get_package_share_directory("go2_localization")

    world       = os.path.join(go2_bringup_pkg, "worlds", "inspection_world.world")
    urdf        = os.path.join(champ_description_pkg, "urdf", "champ.urdf.xacro")
    joints_yaml = os.path.join(champ_config_pkg, "config", "joints", "joints.yaml")
    links_yaml  = os.path.join(champ_config_pkg, "config", "links",  "links.yaml")
    gait_yaml   = os.path.join(champ_config_pkg, "config", "gait",   "gait.yaml")
    ekf_yaml    = os.path.join(go2_localization_pkg, "config", "ekf.yaml")

    robot_description = {"robot_description": Command(["xacro ", urdf])}

    # ── GAZEBO_MODEL_PATH: lets Gazebo resolve model://room ──────────────────
    models_dir        = os.path.join(go2_bringup_pkg, "models")
    existing_gmp      = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_model_path = (models_dir + ":" + existing_gmp) if existing_gmp else models_dir

    # =========================================================================
    # t=0s: Gazebo + robot_state_publisher
    # =========================================================================
    gzserver = ExecuteProcess(
        cmd=["gzserver", "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so", world],
        additional_env={"GAZEBO_MODEL_PATH": gazebo_model_path},
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # =========================================================================
    # t=2s: sim_machine_emulator + inspection_report_gen
    # Both can start early — they only listen/publish to topics, no robot needed.
    # =========================================================================
    sim_machine_emulator = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting sim_machine_emulator..."),
            Node(
                package="go2_sim",
                executable="sim_machine_emulator",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    inspection_report_gen = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting inspection_report_gen..."),
            Node(
                package="go2_report",
                executable="inspection_report_gen",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"output_dir": "/tmp/go2_inspection_reports"},
                    {"mission_name": "Meridian_Platform_Alpha"},
                ],
            ),
        ],
    )

    # =========================================================================
    # t=4s: Spawn robot
    # =========================================================================
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[inspection.launch] Spawning robot into Gazebo..."),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=["-entity", "go1", "-topic", "/robot_description",
                           "-x", "0", "-y", "0", "-z", "0.6"],
            ),
        ],
    )

    # =========================================================================
    # t=6s: go2_hal_node
    # Bridges /go2/gazebo/joint_states and /go2/gazebo/imu to canonical Go2
    # topic names so all downstream nodes are hardware-agnostic.
    # =========================================================================
    go2_hal = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting go2_hal_node..."),
            Node(
                package="go2_hal",
                executable="go2_hal_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    # =========================================================================
    # t=9s: champ_bringup (quadruped controller, /odom/ground_truth)
    # =========================================================================
    champ_bringup = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting champ_bringup..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(champ_bringup_pkg, "launch", "bringup.launch.py")
                ),
                launch_arguments={
                    "use_sim_time":     "true",
                    "gazebo":           "true",
                    "description_path": urdf,
                    "joints_map_path":  joints_yaml,
                    "links_map_path":   links_yaml,
                    "gait_config_path": gait_yaml,
                }.items(),
            ),
        ],
    )

    # =========================================================================
    # t=11s: EKF (robot_localization)
    # Fuses /go2/odom + /go2/imu from go2_hal_node into a smooth pose estimate.
    # publish_tf=False to avoid conflicting with champ's odom->base_link TF.
    # Output available on /odometry/filtered for future Nav2 integration.
    # =========================================================================
    ekf_node = TimerAction(
        period=11.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting EKF (robot_localization)..."),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True, "publish_tf": False}],
            ),
        ],
    )

    # =========================================================================
    # t=14s: Load joint_states_controller
    # =========================================================================
    load_joint_states = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg="[inspection.launch] Loading joint_states_controller..."),
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "joint_states_controller", "--set-state", "active"],
                output="screen",
            ),
        ],
    )

    # =========================================================================
    # t=16s: Load joint_group_effort_controller
    # =========================================================================
    load_effort = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg="[inspection.launch] Loading joint_group_effort_controller..."),
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "joint_group_effort_controller", "--set-state", "active"],
                output="screen",
            ),
        ],
    )

    # =========================================================================
    # t=17s: lidar_sim_node
    # Publishes /go2/scan at 10 Hz by ray-casting against the room walls using
    # live robot pose from /odom/ground_truth. Kill this (or use inject_fault.sh)
    # to simulate a LiDAR sensor failure mid-mission.
    # =========================================================================
    lidar_sim = TimerAction(
        period=17.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting lidar_sim_node..."),
            Node(
                package="go2_sim",
                executable="lidar_sim_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    # =========================================================================
    # t=19s: topic_health_monitor
    # Runs as an independent process from mission_bt_node by design — a crash
    # in the planner must not disable the safety watchdog.
    # Detects /go2/scan silence within 2s and publishes FaultEvent on
    # /mission/fault_event, which mission_bt_node receives to enter degraded mode.
    # =========================================================================
    topic_health_monitor = TimerAction(
        period=19.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting topic_health_monitor..."),
            Node(
                package="go2_health_monitor",
                executable="topic_health_monitor",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    # =========================================================================
    # t=22s: mission_bt_node
    # Waits for /odom/ground_truth before beginning patrol.
    # =========================================================================
    mission_bt_node = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting mission_bt_node..."),
            Node(
                package="go2_mission",
                executable="mission_bt_node",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"enable_groot2": enable_groot2},
                ],
            ),
        ],
    )

    # =========================================================================
    # t=25s: perception_node (obstacle proximity via /odom/ground_truth)
    # =========================================================================
    perception_node = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg="[inspection.launch] Starting perception_node..."),
            Node(
                package="go2_perception",
                executable="perception_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    return LaunchDescription([
        declare_groot2,
        # t=0
        gzserver,
        gzclient,
        robot_state_publisher,
        # t=2
        sim_machine_emulator,
        inspection_report_gen,
        # t=4
        spawn_robot,
        # t=6
        go2_hal,
        # t=9
        champ_bringup,
        # t=11
        ekf_node,
        # t=14
        load_joint_states,
        # t=16
        load_effort,
        # t=17
        lidar_sim,
        # t=19
        topic_health_monitor,
        # t=22
        mission_bt_node,
        # t=25
        perception_node,
    ])
