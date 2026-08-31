"""Jetson: image_capture_node only. Assumes the RealSense camera (Stage 1's
existing launch, not touched here) is already publishing color/camera_info."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ee_calibration'), 'config', 'jetson_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='ee_calibration',
            executable='image_capture_node',
            name='image_capture_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
