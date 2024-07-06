#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'warehouse.model'
    world = os.path.join(get_package_share_directory('grobot_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('grobot_gazebo'), 'launch')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    share_dir = get_package_share_directory('grobot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'R1V2.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': use_sim_time}
        ],
        # arguments=[robot_urdf]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': use_sim_time}
        ],
        # arguments=[robot_urdf]

    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'R1V2',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    calibration_node = Node(package='grobot_node',
                          executable='lidar_calibration',
                          name='lidar_calibration',
                          output='screen',
                          )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        robot_state_publisher_node,
        joint_state_publisher_node,
        urdf_spawn_node,
        calibration_node
    ])
