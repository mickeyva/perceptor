"""
MPU6050 IMU Launch Configuration

This launch file configures and starts the MPU6050 6-axis IMU sensor for the Perceptor robot.
It initializes the MPU6050 using the ros2_mpu6050 driver, providing inertial measurement data
for sensor fusion, navigation, and autonomous operation.

The IMU node publishes calibrated accelerometer and gyroscope data along with computed
orientation estimates using a complementary filter. This data is essential for improved
odometry through sensor fusion with wheel encoders.

MPU6050 SPECIFICATIONS:
- Model: MPU6050 (6-axis IMU: 3-axis accelerometer + 3-axis gyroscope)
- Accelerometer Range: ±2g (configurable: ±2g, ±4g, ±8g, ±16g)
- Gyroscope Range: ±250°/s (configurable: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s)
- Sample Rate: 100Hz (10ms update interval)
- Communication: I2C at address 0x68
- Connection: /dev/i2c-1 (Raspberry Pi I2C bus)

CONFIGURATION OPTIMIZED FOR PERCEPTOR:
- I2C Device: /dev/i2c-1
- I2C Address: 0x68 (MPU6050 default)
- Frame ID: imu_link
- Digital Low Pass Filter: 260Hz/256Hz (accelerometer/gyroscope)
- Clock Source: Internal 8MHz oscillator
- Calibration: Pre-calibrated offsets for level mounting

The ros2_mpu6050 driver provides:
- Hardware initialization and configuration
- Sensor calibration and offset correction
- Complementary filter for orientation estimation
- Standard ROS2 IMU message publishing (/imu/mpu6050)

Node launched (via ros2_mpu6050 package):
- mpu6050_sensor: Official MPU6050 driver
  * Publishes: /imu/mpu6050 (sensor_msgs/Imu)
  * Hardware: MPU6050 via I2C interface
  * Features: Calibrated data, orientation estimation, covariance matrices

Command-line equivalent:
    ros2 run ros2_mpu6050 ros2_mpu6050 --ros-args \
        --params-file /path/to/params.yaml \
        -r __ns:=/ \
        -p use_sim_time:=false

Usage:
    # Recommended (via Perceptor wrapper):
    ros2 launch perceptor mpu6050.launch.py

    # Direct launch (alternative):
    ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py

    # With custom parameters:
    ros2 launch perceptor mpu6050.launch.py param_file:=/path/to/custom_params.yaml

Integration:
    # Launch with main robot system:
    ros2 launch perceptor launch_robot.launch.py enable_imu:=true

    # Launch for sensor fusion:
    ros2 launch perceptor mpu6050.launch.py
    ros2 launch robot_localization ekf_node --ros-args --params-file config/ekf_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate the launch description for the MPU6050 IMU sensor.

    Uses the official ros2_mpu6050 launch file with proper configuration
    for the MPU6050 6-axis IMU (I2C interface, calibrated parameters).

    Returns:
        LaunchDescription: Complete launch configuration for MPU6050 IMU operation
    """

    # Launch configuration for parameter file
    param_file = LaunchConfiguration('param_file')
    
    # Default parameter file from ros2_mpu6050 package
    default_param_file = os.path.join(
        get_package_share_directory('ros2_mpu6050'),
        'config',
        'params.yaml'
    )

    # Parameter file argument declaration
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to MPU6050 parameter file with calibration data'
    )

    # Use the official ros2_mpu6050 launch file from ros2_mpu6050 package
    # This is the recommended approach for MPU6050 IMU integration
    mpu6050_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py'
        )]),
        launch_arguments={
            'param_file': param_file
        }.items()
    )

    return LaunchDescription([
        param_file_arg,      # Parameter file configuration
        mpu6050_launch       # MPU6050 IMU sensor node
    ])