import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'log_level': 'error',
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    ld = LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/imu', description='IMU topic'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic'),
        DeclareLaunchArgument('unite_imu_method', default_value='2', description='0-None, 1-copy (use if imu topics stop being published), 2-linear_interpolation.'),  

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),
        
    ])

    # Launch camera driver. Terminal way is: ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true [etc.]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),'launch/rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'rgb_camera.profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30',
        }.items()
    )

    rtabmap_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings)

    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'])

    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)

    # Compute quaternion of the IMU
    madgwick = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', LaunchConfiguration('imu_topic'))])
    
    icp_server = Node(
            package='img_utilities', executable='icp_server', output='screen',
    )

    pc_fuser = Node(
            package='img_utilities', executable='pc_fuser', output='screen',
    )

    display_node = Node(
            package='img_utilities', executable='display_node', output='screen',
    )

    volume_estimator = Node(
            package='img_utilities', executable='volume_estimator', output='screen',
    )
    
#     imu_node = Node(
#         package='robot_localization',
#         executable='ekf_node', 
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[os.path.join(get_package_share_directory("img_utilities"), 'config', 'params_ekf_node.yaml')],
#         remappings=[]
#   )

#     pc_snap = Node(
#         package='img_utilities',
#         executable='pc_snap',
#         name='pointcloud_saver',
#         output='screen',
#         # parameters=[{
#         #     'use_sim_time': LaunchConfiguration('use_sim_time'),
#         #     'base_link_frame': 'base_link',
#         #     'odom_frame': 'odom',
#         #     'world_frame': 'odom'
#         # }]
#     )



    ld.add_action(camera)
    ld.add_action(madgwick)
    ld.add_action(rtabmap_odom)
    ld.add_action(rtabmap_slam)
    ld.add_action(rtabmap_viz)
    ld.add_action(icp_server)
    ld.add_action(pc_fuser)
    ld.add_action(display_node) # volume_estimator

#   ld.add_action(pc_snap)
#   ld.add_action(imu_node)

    return ld 
