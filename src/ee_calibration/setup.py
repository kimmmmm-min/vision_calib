import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ee_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robottory',
    maintainer_email='seungdomin331@gmail.com',
    description='EE-as-fiducial camera-to-robot-base calibration for UR10 CB2 (no calibration board)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Desktop
            'joint_state_bridge_node = ee_calibration.joint_state_bridge_node:main',
            'trajectory_bridge_node = ee_calibration.trajectory_bridge_node:main',
            'calibration_point_publisher = ee_calibration.calibration_point_publisher:main',
            'pose_sampler_node = ee_calibration.pose_sampler_node:main',
            # Jetson
            'image_capture_node = ee_calibration.image_capture_node:main',
            'ee_click_tool = ee_calibration.ee_click_tool:main',
            'solve_calibration = ee_calibration.solve_calibration:main',
        ],
    },
)
