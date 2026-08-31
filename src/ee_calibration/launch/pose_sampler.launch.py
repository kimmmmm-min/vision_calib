"""Desktop: pose_sampler_node only (assumes desktop_bridge.launch.py and
ur_moveit_config's ur_moveit.launch.py are already running)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ee_calibration'), 'config', 'desktop_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='ee_calibration',
            executable='pose_sampler_node',
            name='pose_sampler_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
