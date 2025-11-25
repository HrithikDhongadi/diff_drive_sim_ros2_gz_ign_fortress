#!/usr/bin/env python3
"""
SLAM launch file for map creation with slam_toolbox.
Launches online async SLAM for real-time map generation.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for SLAM map creation."""
    
    # Get package directories
    pkg_share = get_package_share_directory('diff_drive_sim')
    slam_toolbox_pkg_share = get_package_share_directory('slam_toolbox')
    
    # Define paths
    slam_config_file = os.path.join(
        pkg_share, 
        'config', 
        'mapper_params_online_async.yaml'
    )
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_config = DeclareLaunchArgument(
        'slam_config_file',
        default_value=slam_config_file,
        description='Full path to SLAM configuration file'
    )
    
    # SLAM Toolbox online async launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_config_file'),
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_slam_config,
        slam_launch,
    ])
    
    return ld
