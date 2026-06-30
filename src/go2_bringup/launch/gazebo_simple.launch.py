import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    gz_pkg = launch_ros.substitutions.FindPackageShare("champ_gazebo").find("champ_gazebo")
    desc_pkg = launch_ros.substitutions.FindPackageShare("champ_description").find("champ_description")
    
    world = os.path.join(gz_pkg, "worlds/default.world")
    urdf = os.path.join(desc_pkg, "urdf/champ.urdf.xacro")

    robot_description = {"robot_description": Command(["xacro ", urdf])}

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gzserver", "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so", world],
            output="screen"
        ),
        ExecuteProcess(cmd=["gzclient"], output="screen"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}]
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=["-entity", "go1", "-topic", "/robot_description",
                       "-x", "0", "-y", "0", "-z", "0.6"]
        ),
    ])
