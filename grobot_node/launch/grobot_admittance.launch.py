from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    velocity_controller_node = Node(package='grobot_node',
                          executable='velocity_manager',
                          name='velocity_manager',
                          output='screen',
                          )
    
    admittance_interfaces_node = Node(package='grobot_navigation2',
                          executable='admittance_interfaces',
                          name='admittance_interfaces',
                          output='screen',
                          )

    force_publisher_node = Node(package='grobot_node',
                          executable='force_publisher_node',
                          name='force_publisher_node',
                          output='screen',
                          )

    return LaunchDescription([
        force_publisher_node,
        admittance_interfaces_node,
        velocity_controller_node

    ])
