#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions


def generate_launch_description():
  grobot_mcu_parameter = LaunchConfiguration(
    'grobot_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('grobot_bringup'),
      'param/grobot_mcu.yaml'
    )
  )

  grobot_lidar_parameter = LaunchConfiguration(
    'grobot_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('grobot_bringup'),
      'param/grobot_lidar.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  grobot_description_dir = LaunchConfiguration(
    'grobot_description_dir',
    default=os.path.join(
      get_package_share_directory('grobot_description'),
      'launch'
    )
  )
  
  grobot_imu_launch_file = os.path.join(
      get_package_share_directory('grobot_sensor'),
      'launch',
      'grobot_imu.launch.py'
  )

  return LaunchDescription([
    
    launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("grobot_bringup"), 'param', 'ekf.yaml')],
    ),
    
    DeclareLaunchArgument(
      'grobot_mcu_parameter',
      default_value=grobot_mcu_parameter
    ),

    DeclareLaunchArgument(
      'grobot_lidar_parameter',
      default_value=grobot_lidar_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/grobot_mcu.launch.py']),
      launch_arguments={'grobot_mcu_parameter': grobot_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/grobot_lidar.launch.py']),
      launch_arguments={'grobot_lidar_parameter': grobot_lidar_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([grobot_description_dir, '/grobot_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(grobot_imu_launch_file),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])
