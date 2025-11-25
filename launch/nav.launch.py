#!/usr/bin/env python3
"""
Navigation stack launch file for diff_drive robot.
Launches Nav2 localization and navigation.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for Nav2 stack."""
    
    # Get package directories
    pkg_share = get_package_share_directory('diff_drive_sim')
    nav2_pkg_share = get_package_share_directory('nav2_bringup')
    
    # Define paths
    map_file = os.path.join(pkg_share, 'maps', 'closed_map_v2.yaml')
    nav_params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to the map file to load'
    )
    
    declare_nav_params = DeclareLaunchArgument(
        'nav_params_file',
        default_value=nav_params_file,
        description='Full path to the navigation parameters file'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )
    
    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_nav_params,
        declare_autostart,
        localization_launch,
        navigation_launch,
    ])
    
    return ld
