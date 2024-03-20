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
from launch_ros.actions import Node


def generate_launch_description():
  grobot_mcu_parameter = LaunchConfiguration(
    'grobot_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('grobot_bringup'),
      'param/grobot_mcu.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'grobot_mcu_parameter',
      default_value=grobot_mcu_parameter
    ),

    Node(
      package='grobot_bringup',
      executable='grobot_mcu_node',
      name='grobot_mcu_node',
      output='screen',
      emulate_tty=True,
      parameters=[grobot_mcu_parameter],
      namespace='',
    )
  ])
