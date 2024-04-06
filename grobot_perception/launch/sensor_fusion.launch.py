
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    camera_config_file = "Camera_config.yaml"
    lidar_config_file = "Lidar_config.yaml"
    node_config_file = "Node_config.yaml"
    calibration_config_file = "CalibrationSettings.yaml"

    camera_config = os.path.join(get_package_share_directory("sensor_fusion"), "config", camera_config_file)
    lidar_config = os.path.join(get_package_share_directory("sensor_fusion"), "config", lidar_config_file)
    node_config = os.path.join(get_package_share_directory("sensor_fusion"), "config", node_config_file)
    calibration_config = os.path.join(get_package_share_directory("sensor_fusion"), "config", calibration_config_file)

    sensor_fusion_node = Node(
        package='sensor_fusion',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        parameters=[
        {"camera_yaml_config_path" : camera_config},
        {"lidar_yaml_config_path" : lidar_config},
        {"node_yaml_config_path": node_config},
        {"calibration_yaml_config_path": calibration_config}
        ]
    )

    return LaunchDescription([sensor_fusion_node])