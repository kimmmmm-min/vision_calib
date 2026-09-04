from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee_calibration_static_tf',
            arguments=[
                '0.462782', '0.906114', '1.242130',
                '0.241219', '0.841183', '-0.472785', '-0.103434',
                'base', 'camera_color_optical_frame',
            ],
        ),
    ])
