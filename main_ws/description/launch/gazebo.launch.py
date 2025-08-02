#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2.",
        )
    )

    # Initialize Arguments
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get the package directory
    pkg_share = FindPackageShare("description")
    
    # Get the URDF file path
    urdf_file = os.path.join(get_package_share_directory("description"), "urdf", "docky.urdf")
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        condition=IfCondition(use_sim),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "docky"],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # Launch RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("description"), "rviz", "urdf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes) 