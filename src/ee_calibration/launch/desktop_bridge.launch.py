"""Desktop bridge nodes, robot TF, and the physical calibration-point TF.

Run this BEFORE ur_moveit_config's ur_moveit.launch.py. This does not move the
robot by itself -- it only wires urcb2_driver's joint states into TF and
stands up the FollowJointTrajectory action server that MoveIt2's
moveit_simple_controller_manager expects.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    params_file = LaunchConfiguration('params_file')

    default_params = os.path.join(
        get_package_share_directory('ee_calibration'), 'config', 'desktop_params.yaml')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']), ' ',
        # Not used for real communication (urcb2_driver owns the hardware
        # link) -- only needed because the xacro requires it to generate the
        # (unused) URCap control script.
        'robot_ip:=xxx.yyy.zzz.www', ' ',
        'name:=ur', ' ',
        'ur_type:=', ur_type, ' ',
        'script_filename:=ros_control.urscript', ' ',
        'input_recipe_filename:=rtde_input_recipe.txt', ' ',
        'output_recipe_filename:=rtde_output_recipe.txt', ' ',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur10'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='ee_calibration',
            executable='joint_state_bridge_node',
            name='joint_state_bridge_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='ee_calibration',
            executable='trajectory_bridge_node',
            name='trajectory_bridge_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='ee_calibration',
            executable='calibration_point_publisher',
            name='calibration_point_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])
