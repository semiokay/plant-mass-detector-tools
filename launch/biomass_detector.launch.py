import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu', description='IMU topic'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic'),
    ])

    # ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/rs_launch.py')
        ),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '0',
            'pointcloud.enable': 'true'
        }.items()
    )

    pc_subscriber = Node(
        package='img_utilities',
        executable='pc_subscriber',
        name='pc_subscriber',
        output='screen',
        # parameters=[{
        #     'use_sim_time': LaunchConfiguration('use_sim_time'),
        #     'base_link_frame': 'base_link',
        #     'odom_frame': 'odom',
        #     'world_frame': 'odom'
        # }]
    )
  
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters= [{
            'use_mag': False
        }],
        remappings= [
            ("/imu/data_raw", LaunchConfiguration('imu_topic'))
        ]
    )

    imu_node = Node(
        package='robot_localization',
        executable='ekf_node', #'ekf_localization_node',
        name='ekf_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu0': LaunchConfiguration('imu_topic'),
            'odom0': LaunchConfiguration('odom_topic'),
            'imu0_config': [True, True, False, False, False, True],
            'odom0_config': [True, True, False, False, False, False],
            'freq': 30.0,
            'sensor_timeout': 1.0,
            'base_link_frame': 'base_link',
            'odom_frame': 'odom',
            'world_frame': 'odom'
        }]
    )

 

    # ld.add_action(camera_launch)
    # ld.add_action(pc_subscriber)
    # ld.add_action(madgwick)
    ld.add_action(imu_node)


    return ld
