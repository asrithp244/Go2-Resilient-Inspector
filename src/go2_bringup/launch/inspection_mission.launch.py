"""
inspection_mission.launch.py
=============================
Single launch file for the full Go2 resilient inspection system.

Usage:
  # Happy path (no fault injection):
  ros2 launch go2_bringup inspection_mission.launch.py

  # With fault injection at 60 seconds:
  ros2 launch go2_bringup inspection_mission.launch.py inject_fault:=true fault_delay_sec:=60.0

  # With Groot2 BT visualization:
  ros2 launch go2_bringup inspection_mission.launch.py enable_groot2:=true

Launch arguments:
  inject_fault      (bool, default false) — arm the fault injection timer
  fault_delay_sec   (float, default 60.0) — seconds after launch before injecting
  fault_subsystem   (str, default lidar)  — which subsystem to fault
  use_sim_time      (bool, default true)  — use Gazebo simulated clock
  enable_groot2     (bool, default false) — launch Groot2 BT publisher
  log_level         (str, default info)   — rclcpp log level
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    inject_fault_arg = DeclareLaunchArgument(
        "inject_fault", default_value="false",
        description="If true, kill the lidar topic after fault_delay_sec.")

    fault_delay_arg = DeclareLaunchArgument(
        "fault_delay_sec", default_value="60.0",
        description="Seconds after launch to inject the fault.")

    fault_subsystem_arg = DeclareLaunchArgument(
        "fault_subsystem", default_value="lidar",
        description="Subsystem to fault: lidar, camera, can_bus, imu.")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulated clock.")

    enable_groot2_arg = DeclareLaunchArgument(
        "enable_groot2", default_value="false",
        description="Enable Groot2 BT live visualization on port 1667.")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="rclcpp log level (debug, info, warn, error).")

    use_sim_time = LaunchConfiguration("use_sim_time")
    inject_fault  = LaunchConfiguration("inject_fault")
    fault_delay   = LaunchConfiguration("fault_delay_sec")
    enable_groot2 = LaunchConfiguration("enable_groot2")
    log_level     = LaunchConfiguration("log_level")

    # ── Package paths ─────────────────────────────────────────────────────────
    bringup_share    = FindPackageShare("go2_bringup")
    mission_share    = FindPackageShare("go2_mission")
    loc_share        = FindPackageShare("go2_localization")

    ekf_config       = PathJoinSubstitution([loc_share,   "config", "ekf.yaml"])
    mission_config   = PathJoinSubstitution([mission_share, "config", "mission_params.yaml"])
    bt_xml           = PathJoinSubstitution([mission_share, "behavior_trees", "inspection_mission.xml"])
    nav2_params      = PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"])

    # ── Nodes ─────────────────────────────────────────────────────────────────

    go2_hal_node = Node(
        package="go2_hal",
        executable="go2_hal_node",
        name="go2_hal_node",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    health_monitor_node = Node(
        package="go2_health_monitor",
        executable="topic_health_monitor",
        name="topic_health_monitor",
        parameters=[{
            "use_sim_time": use_sim_time,
            "lidar_timeout_sec": 2.0,
            "camera_timeout_sec": 3.0,
            "can_timeout_sec": 5.0,
            "imu_timeout_sec": 1.0,
            "check_rate_hz": 10.0,
        }],
        output="screen",
    )

    sim_machine_emulator_node = Node(
        package="go2_sim",
        executable="sim_machine_emulator",
        name="sim_machine_emulator",
        parameters=[{
            "use_sim_time": use_sim_time,
            "publish_rate_hz": 2.0,
            "noise_enabled": True,
        }],
        output="screen",
    )

    perception_node = Node(
        package="go2_perception",
        executable="perception_node",
        name="perception_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "use_yolo": False,
            "confidence_threshold": 0.7,
        }],
        output="screen",
    )

    mission_bt_node = Node(
        package="go2_mission",
        executable="mission_bt_node",
        name="mission_bt_node",
        parameters=[
            mission_config,
            {
                "use_sim_time": use_sim_time,
                "bt_xml_file": bt_xml,
                "enable_groot2": enable_groot2,
            }
        ],
        output="screen",
    )

    report_gen_node = Node(
        package="go2_report",
        executable="inspection_report_gen",
        name="inspection_report_gen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "output_dir": "/tmp/go2_inspection_reports",
            "auto_generate_on_complete": True,
            "mission_name": "Meridian_Platform_Alpha",
        }],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    teleop_node = Node(
        package="go2_teleop",
        executable="joy_teleop_node",
        name="joy_teleop_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_linear_speed": 0.8,
            "max_angular_speed": 1.0,
            "deadband": 0.05,
            "autonomous_mode": False,
        }],
        output="screen",
    )

    # ── Fault injection (conditional on inject_fault:=true) ───────────────────
    # Uses scripts/inject_fault.sh to kill the lidar publisher after a delay.
    # This simulates the real-world scenario of a sensor failure mid-mission.
    fault_injector = TimerAction(
        period=fault_delay,
        actions=[
            LogInfo(msg=["[FAULT INJECTION] Killing /go2/scan publisher in 1 second..."]),
            ExecuteProcess(
                cmd=["ros2", "topic", "pub", "--once",
                     "/mission/inject_fault", "std_msgs/msg/Bool", "{data: true}"],
                output="screen",
            ),
        ],
        condition=IfCondition(inject_fault),
    )

    # ── Nav2 bringup ──────────────────────────────────────────────────────────
    # Include the Nav2 launch (simplified — uses bringup package's params)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    return LaunchDescription([
        # Arguments
        inject_fault_arg,
        fault_delay_arg,
        fault_subsystem_arg,
        use_sim_time_arg,
        enable_groot2_arg,
        log_level_arg,

        # Core system nodes (order matters for dependency resolution)
        go2_hal_node,
        ekf_node,
        health_monitor_node,       # must be independent of mission BT
        sim_machine_emulator_node,
        perception_node,
        nav2_bringup,
        mission_bt_node,
        report_gen_node,
        joy_node,
        teleop_node,

        # Conditional fault injection
        fault_injector,
    ])
