#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('grobot_navigation2'),
            'map',
            'map0521.yaml'))

    # param_file_name = 'burger.yaml'
    param_file_name = 'grobot_param.yaml'
    # param_file_name = 'grobot_cart_param.yaml'
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('grobot_navigation2'),
            'param',
            param_file_name))
    
    grobot_admittance_dir = LaunchConfiguration(
    'grobot_admittance_dir',
    default=os.path.join(
      get_package_share_directory('grobot_node'),
      'launch'
    )
  )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    
    waypoint_yaml_path = os.path.join(
        get_package_share_directory('grobot_navigation2'),
        'param',
        'path_distance_basement.yaml')
    
    waypoint_basement_yaml_path = os.path.join(
        get_package_share_directory('grobot_navigation2'),
        'param',
        'waypoint_basement.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([grobot_admittance_dir, '/grobot_admittance.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        #     ),
            
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        Node(
            package='grobot_navigation2',
            executable='waypoint_dynamic_programming',
            name='waypoint_dynamic_programming',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'waypoint_yaml_path': waypoint_yaml_path},
            ],
        ),
        
        Node(
            package='grobot_navigation2',
            executable='waypoint_follower',
            name='waypoint_follower_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'waypoint_basement_yaml_path': waypoint_basement_yaml_path},
            ],
        ),
    ])
