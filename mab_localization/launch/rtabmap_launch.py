import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_name = "mab_localization"
    pkg_share = get_package_share_directory(pkg_name)
    laser_settings_file = os.path.join(pkg_share, 'resource', 'laser.yaml')
    slam_settings_file = os.path.join(pkg_share, 'resource', 'slam_toolbox_params.yaml')
    ekf_settings_file = os.path.join(pkg_share, 'resource', 'ekf.yaml')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'resource', 'silver_badger_slam.rviz')],
        # parameters=[{'ros__parameters/use_sim_time':True}]
        # parameters=[{'ros__parameters':{
        #     'use_sim_time': True,
        # }}]
    )
    
    rtabmap = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'approx_sync': True,
            'odom_frame_id': 'odom',
            'publish_tf': False,
            # 'odom_frame_id': 'odom_turtlebot3',
            # 'queue_size': 1000,
            # 'wait_for_transform': 1.0,
            # 'approx_sync_max_interval': 0.5,
            # 'sensor_data_compression_format': 'jpeg',
            # 'topic_queue_size': 1000,
            'Odom/Strategy': '0', # Frame to map or Frame to frame
            'Vis/CorType': '0', # Features Matching or Optical flow
            'OdomF2M/MaxSize': '1000', # Max features
            'Vis/MaxFeatures': '700', # Max features extracted from image
        }],
        remappings=[
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info')
            # ('/rgb/image', '/waffle_cam/color/image_raw'),
            # ('/depth/image', '/waffle_cam/depth/image_raw'),
            # ('/rgb/camera_info', '/waffle_cam/color/camera_info')
        ]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'approx_sync': True,
            'odom_frame_id': 'odom',
            # 'queue_size': 1000,
            # 'wait_for_transform': 1.0,
            # 'approx_sync_max_interval': 0.5,
            # 'Odom/Strategy': '1', # Frame to map or Frame to frame
            # 'Vis/CorType': '1', # Features Matching or Optical flow
            # 'OdomF2M/MaxSize': '1000', # Max features
            # 'Vis/MaxFeatures': '600', # Max features extracted from image
            # 'Grid/3D': 'False',
            # 'subscribe_depth': False,
            # 'subscribe_scan': True,
            # 'Grid/Sensor': '1',
        }],
        remappings=[
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            ('/imu', '/imu/out'),
            ('/scan', '/scan')
            # ('/rgb/image', '/waffle_cam/color/image_raw'),
            # ('/depth/image', '/waffle_cam/depth/image_raw'),
            # ('/rgb/camera_info', '/waffle_cam/color/camera_info')
            # ('/imu', '/imu'),
        ],
        # arguments=['--delete_db_on_start']

    )

    rtabmap_vis = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        output='screen',
    )

    rotate_depth = Node(
        package='mab_localization',
        executable='image_rotate_node',
        name='image_rotate_node',
        output='screen',
    )



    depth_to_laser = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[laser_settings_file],
        remappings=[
            ('depth', '/d435i_camera/depth/image_raw/rotated'),
            ('depth_camera_info', '/d435i_camera/depth/camera_info/rotated'),
            # ('depth', '/d435i_camera/depth/image_raw'),
            # ('depth_camera_info', '/d435i_camera/depth/camera_info'),
            # ('depth', '/waffle_cam/depth/image_raw'),
            # ('depth_camera_info', '/waffle_cam/depth/camera_info'),
            ('scan', '/scan')
        ]
    )

    imu_covariance = Node(
        package='mab_localization',
        executable='imu_covariance_node',
        name='imu_covariance_node',
        output='screen',
    )

    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_settings_file],
    )

    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_settings_file],
        # remappings=[
        #     ('/scan', scan_topic)
        # ]
    )

    map_saver = Node(
        package='mab_localization',
        executable='map_saver',
        name='map_saver',
        output='screen',
    )

    # nav2 = Node(
    #     package='nav2_bringup',
    #     executable='bringup_launch.py',
    #     name='bringup_launch',
    #     output='screen',
    #     parameters=[config_file],
    #     arguments=['--params-file', config_file],
    # )

    # nav2 = Node(
    #     package='nav2_bringup',
    #     executable='bringup_launch.py',
    #     name='bringup_launch',
    #     output='screen',
    #     parameters=[nav2_settings_file],
    #     arguments=['--params-file', nav2_settings_file],
    # ),

    # nav2 = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time,
    #             # 'params_file': nav2_settings_file,
    #             # 'autostart': True,
    #             'map': '/map',
    #         }.items()
    # )

    velocity_relay = Node(
        package='mab_localization',
        executable='velocity_relay',
        name='velocity_relay',
        output='screen',
    )


    return LaunchDescription([
        # use_sim_time_arg,
        # nav2,
        rviz,
        rtabmap,
        # rtabmap_vis,
        # rtabmap_slam,
        rotate_depth,
        depth_to_laser,
        imu_covariance,
        ekf_filter,
        slam,
        velocity_relay,
        # map_saver
    ])
