"""
Sensor Fusion Launch Configuration

This launch file configures and starts the sensor fusion system for the Perceptor robot.
It combines wheel odometry from the Create robot base with IMU data from the MPU6050
sensor using an Extended Kalman Filter (EKF) for improved localization accuracy.

SENSOR FUSION CONFIGURATION:
- Primary Input: Wheel odometry from Create robot (/odom)
- Secondary Input: IMU data from MPU6050 (/imu/mpu6050)
- Fusion Algorithm: Extended Kalman Filter (robot_localization package)
- Output: Fused odometry (/odometry/filtered)
- Frame Configuration: 2D mode optimized for ground robots

The EKF provides:
- Improved odometry accuracy by combining multiple sensor sources
- Better handling of wheel slip and encoder drift
- Enhanced angular velocity estimation from IMU gyroscope
- Robust state estimation for navigation and SLAM applications

Components launched:
1. EKF Filter Node: robot_localization EKF implementation
   * Subscribes: /odom (nav_msgs/Odometry) - wheel odometry
   * Subscribes: /imu/mpu6050 (sensor_msgs/Imu) - IMU data
   * Publishes: /odometry/filtered (nav_msgs/Odometry) - fused odometry
   * Publishes: /tf (tf2_msgs/TFMessage) - odom→base_link transform

Data Flow:
    Create Encoders → /odom ↘
                            → EKF → /odometry/filtered → Navigation/SLAM
    MPU6050 IMU → /imu/mpu6050 ↗

Usage:
    # Standalone sensor fusion (requires IMU and odometry sources)
    ros2 launch perceptor sensor_fusion.launch.py

    # With custom EKF parameters
    ros2 launch perceptor sensor_fusion.launch.py params_file:=/path/to/custom_ekf_config.yaml

    # Integrated with main robot system
    ros2 launch perceptor launch_robot.launch.py enable_sensor_fusion:=true

Prerequisites:
    - IMU sensor must be running: ros2 launch perceptor mpu6050.launch.py
    - Robot base must be running: ros2 launch perceptor launch_robot.launch.py
    - Static transform base_link→imu_link must be published
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate the launch description for the sensor fusion system.

    Uses the robot_localization EKF node with pre-configured parameters
    optimized for Create robot + MPU6050 IMU sensor fusion.

    Returns:
        LaunchDescription: Complete launch configuration for sensor fusion operation
    """

    # Package name configuration
    package_name = 'perceptor'

    # EKF configuration file path
    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf_config.yaml'
    )

    # Launch configuration for parameter file
    params_file = LaunchConfiguration('params_file')

    # Parameter file argument declaration
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to EKF configuration file with sensor fusion parameters'
    )

    # EKF Filter Node
    # Extended Kalman Filter for sensor fusion of wheel odometry and IMU data
    # Functionality: Combines /odom and /imu/mpu6050 to produce /odometry/filtered
    # Dependencies: Requires running IMU sensor and robot base odometry
    ekf_node = Node(
        package='robot_localization',        # Robot localization package
        executable='ekf_node',               # EKF filter executable
        name='ekf_filter_node',              # Unique node name
        output='screen',                     # Display output in terminal
        parameters=[params_file],            # EKF configuration parameters
        remappings=[
            # Input remappings (if needed)
            # ('/odometry/filtered', '/odometry/filtered'),  # Default output topic
        ]
    )

    return LaunchDescription([
        params_file_arg,    # Parameter file configuration
        ekf_node,           # EKF sensor fusion node
    ])