#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'R1V2.xacro'
    urdf_file_name = 'grobot_cart.xacro'

    share_dir = get_package_share_directory('grobot_description')
    # xacro_file = os.path.join(share_dir, 'urdf', 'R1V2.xacro')
    xacro_file = os.path.join(share_dir, 'urdf', 'grobot_cart.xacro')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    
    urdf = os.path.join(
        get_package_share_directory('grobot_description'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': use_sim_time}
        ],
            # arguments=[urdf]) ## origin
            arguments=[robot_urdf])
    ])
