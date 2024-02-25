import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node


def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(
        get_package_share_directory('hyrian_navigation2'), 
        'behavior_tree')

    # Parameters
    bt_xml = LaunchConfiguration(
        'bt_xml', 
        default=bt_xml_dir+'/hyrian_bt.xml')

    behavior_tree = Node(
        package='hyrian_navigation2',
        executable='bt_main',
        parameters=[{'bt_xml': bt_xml}],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(behavior_tree)

    return ld