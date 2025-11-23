#!/usr/bin/env python3
"""
Main launch file for diff_drive robot simulation in Gazebo Fortress.
Launches Gazebo, spawns the robot, and starts the robot state publisher.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for diff_drive robot simulation."""
    
    # Get package directory
    pkg_share = get_package_share_directory('diff_drive_sim')
    
    # Define paths
    # urdf_file = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf')
    sdf_file = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.sdf')
    world_file = os.path.join(pkg_share, 'world', 'default.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # Read URDF file
    with open(sdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Launch Gazebo headless (no GUI)'
    )
    
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output for debugging'
    )
    
    # Robot state publisher node - process URDF through xacro first
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0
        }]
    )

    # Gazebo Fortress server
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', world_file, '--verbose'],
        output='screen'
    )
    
    # Gazebo Fortress client (GUI) - only launch if not headless
    gazebo_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
    
    # Spawn robot entity using ign service call
    spawn_entity = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 2 && '
             'ign service -s /world/default/create '
             '--reqtype ignition.msgs.EntityFactory '
             '--reptype ignition.msgs.Boolean '
             '--timeout 1000 '
             '--req \'sdf_filename: "' + sdf_file + '", name: "diff_drive_robot", pose: {position: {z: 0.2}}\''],
        output='screen'
    )
    
    # RViz node
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    # Bridge for ROS2 - sim
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config  # <- point to your YAML list file
        }],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        declare_verbose,
        robot_state_publisher_node,
        # joint_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        ros_gz_bridge,
        rviz_node
    ])
    
    return ld
