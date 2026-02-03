"""
xv11_lidar_python Launch Configuration

"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate the launch description for the xv11_lidar_python sensor.
    Returns:
        LaunchDescription: Complete launch configuration for xv11_lidar_python operation
    """

    xv11_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('xv11_lidar_python'), 'launch', 'xv11_lidar_launch.py'
        )]),
        launch_arguments={
            'serial_port': '/dev/ttyACM0',
            'serial_baudrate': '115200',
            'frame_id': 'laser',
            'inverted': 'false',
        }.items()
    )

    return LaunchDescription([
        xv11_lidar_launch
    ])
