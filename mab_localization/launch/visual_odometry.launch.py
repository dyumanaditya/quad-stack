import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_name = "mab_localization"
    kinematics_odom_pkg_name = "legged_kinematics_odometry"
    description_pkg_name = "mab_description"
    pkg_share = get_package_share_directory(pkg_name)
    kinematics_odom_pkg_share = get_package_share_directory(kinematics_odom_pkg_name)
    description_pkg_share = get_package_share_directory(description_pkg_name)
    ekf_settings_file = os.path.join(pkg_share, 'resource', 'ekf.yaml')
    
    rtabmap = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'approx_sync': True,
            'odom_frame_id': 'odom',
            'publish_tf': True,
            # 'initial_pose': '-2.0 3.5 0 0 0 0',
            'initial_pose': '-2.0 3.5 0.3 0 0 0',
            'publish_null_when_lost': False,
            # 'odom_frame_id': 'odom_turtlebot3',
            # 'queue_size': 1000,
            # 'wait_for_transform': 1.0,
            'approx_sync_max_interval': 0.05,
            # 'sensor_data_compression_format': 'jpeg',
            # 'topic_queue_size': 1000,
            'Odom/Strategy': '0', # Frame to map or Frame to frame
            'Vis/CorType': '0', # Features Matching or Optical flow
            'OdomF2M/MaxSize': '1200', # Max features
            'Vis/MaxFeatures': '700', # Max features extracted from image
            'wait_imu_to_init': True,
            # 'guess_frame_id': 'odom_kinematics',
        }],
        remappings=[
            # ('/rgb/image', '/d435i_camera/color/image_raw/rotated'),
            # ('/depth/image', '/d435i_camera/depth/image_raw/rotated'),
            # ('/rgb/camera_info', '/d435i_camera/color/camera_info/rotated')
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_rect_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            # ('/rgb/image', '/d435i_camera/color/image_raw/stabilized'),
            # ('/depth/image', '/d435i_camera/depth/image_rect_raw/stabilized'),
            # ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            # ('/depth/image', '/d435i_camera/depth/image_raw'),
            # ('/rgb/image', '/waffle_cam/color/image_raw'),
            # ('/depth/image', '/waffle_cam/depth/image_raw'),
            # ('/rgb/camera_info', '/waffle_cam/color/camera_info')
            ('/imu', '/imu/out'),
        ]
    )

    # kinematics_odometry = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(kinematics_odom_pkg_share, 'launch', 'kinematics_odometry.launch.py')),
    #     launch_arguments={'urdf': os.path.join(description_pkg_share, 'urdf', 'silver_badger.urdf')}.items()
    # )

    odom_gt = Node(
        package='mab_utils',
        executable='odom_gt',
        name='odom_gt',
        output='screen',
    )

    odom_2d = Node(
        package='mab_localization',
        executable='odom_2d',
        name='odom_2d',
        output='screen',
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

    # rtabmap_vis = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmap_viz',
    #     name='rtabmapviz',
    #     output='screen',
    # )


    return LaunchDescription([
        rtabmap,
        # odom_gt,
        # odom_2d,
        # kinematics_odometry,
        # imu_covariance,
        # ekf_filter,
        # rtabmap_vis,
    ])
