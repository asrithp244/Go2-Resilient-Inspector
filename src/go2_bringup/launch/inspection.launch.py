"""
inspection.launch.py
====================
Single launch file for the Go2 Resilient Inspector simulation.
Replaces the previous 4-terminal startup sequence.

Startup sequence (timed to avoid race conditions):
  t= 0s  Gazebo server + client + robot_state_publisher
  t= 4s  Spawn robot into Gazebo
  t= 9s  champ_bringup (quadruped controller, EKF, state estimation)
  t=14s  Load joint_states_controller
  t=16s  Load joint_group_effort_controller
  t=19s  mission_bt_node (begins patrol once /odom/ground_truth arrives)

Usage:
  ros2 launch go2_bringup inspection.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # ── Package paths ─────────────────────────────────────────────────────────
    go2_bringup_pkg      = get_package_share_directory('go2_bringup')
    champ_description_pkg = get_package_share_directory('champ_description')
    champ_config_pkg     = get_package_share_directory('champ_config')
    champ_bringup_pkg    = get_package_share_directory('champ_bringup')
    go2_mission_pkg      = get_package_share_directory('go2_mission')

    world      = os.path.join(go2_bringup_pkg, 'worlds', 'inspection_world.world')
    urdf       = os.path.join(champ_description_pkg, 'urdf', 'champ.urdf.xacro')
    joints_yaml = os.path.join(champ_config_pkg, 'config', 'joints', 'joints.yaml')
    links_yaml  = os.path.join(champ_config_pkg, 'config', 'links',  'links.yaml')
    gait_yaml   = os.path.join(champ_config_pkg, 'config', 'gait',   'gait.yaml')

    robot_description = {'robot_description': Command(['xacro ', urdf])}

    # ── t=0s: Gazebo + robot_state_publisher ─────────────────────────────────
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ── t=4s: Spawn robot ─────────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg='[inspection.launch] Spawning robot into Gazebo...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=['-entity', 'go1', '-topic', '/robot_description',
                           '-x', '0', '-y', '0', '-z', '0.6']
            ),
        ]
    )

    # ── t=9s: champ_bringup ───────────────────────────────────────────────────
    champ_bringup = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg='[inspection.launch] Starting champ_bringup...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(champ_bringup_pkg, 'launch', 'bringup.launch.py')
                ),
                launch_arguments={
                    'use_sim_time':    'true',
                    'gazebo':          'true',
                    'description_path': urdf,
                    'joints_map_path':  joints_yaml,
                    'links_map_path':   links_yaml,
                    'gait_config_path': gait_yaml,
                }.items()
            ),
        ]
    )

    # ── t=14s: Load joint_states_controller ──────────────────────────────────
    load_joint_states = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg='[inspection.launch] Loading joint_states_controller...'),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     'joint_states_controller', '--set-state', 'active'],
                output='screen'
            ),
        ]
    )

    # ── t=16s: Load joint_group_effort_controller ─────────────────────────────
    load_effort = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg='[inspection.launch] Loading joint_group_effort_controller...'),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     'joint_group_effort_controller', '--set-state', 'active'],
                output='screen'
            ),
        ]
    )

    # ── t=19s: mission_bt_node ────────────────────────────────────────────────
    mission_bt_node = TimerAction(
        period=19.0,
        actions=[
            LogInfo(msg='[inspection.launch] Starting mission_bt_node...'),
            Node(
                package='go2_mission',
                executable='mission_bt_node',
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        champ_bringup,
        load_joint_states,
        load_effort,
        mission_bt_node,
    ])
