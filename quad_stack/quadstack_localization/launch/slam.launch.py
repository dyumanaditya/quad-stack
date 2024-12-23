import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_name = "mab_localization"
    utils_pkg_name = "mab_utils"
    pkg_share = get_package_share_directory(pkg_name)
    utils_pkg_share = get_package_share_directory(utils_pkg_name)
    laser_settings_file = os.path.join(pkg_share, 'resource', 'laser.yaml')
    slam_settings_file = os.path.join(pkg_share, 'resource', 'slam_toolbox_params.yaml')


    depth_to_laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(utils_pkg_share, 'launch', 'depth_to_laser.launch.py'))
    )

    # depth_to_laser = Node(
    #     package='depthimage_to_laserscan',
    #     executable='depthimage_to_laserscan_node',
    #     name='depthimage_to_laserscan',
    #     output='screen',
    #     parameters=[laser_settings_file],
    #     remappings=[
    #         ('depth', '/d435i_camera/depth/image_raw/rotated'),
    #         ('depth_camera_info', '/d435i_camera/depth/camera_info/rotated'),
    #         # ('depth', '/d435i_camera/depth/image_raw'),
    #         # ('depth_camera_info', '/d435i_camera/depth/camera_info'),
    #         # ('depth', '/d435i_camera/depth/stabilized_image'),
    #         # ('depth_camera_info', '/d435i_camera/depth/stabilized_camera_info'),
    #         # ('depth', '/waffle_cam/depth/image_raw'),
    #         # ('depth_camera_info', '/waffle_cam/depth/camera_info'),
    #         ('scan', '/scan')
    #     ]
    # )

    # SLAM Node
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_settings_file]
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
        arguments=['--delete_db_on_start']

    )

    point_cloud_to_occupancy_grid = Node(
        package='mab_utils',
        executable='point_cloud_to_occupancy',
        name='point_cloud_to_occupancy',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_obstacles',
            'occupancy_grid_topic': '/map_flattened',
            'height_threshold': 0.1,
            'grid_resolution': 0.1,
            'intensity_threshold': 7
        }]
    )

    return LaunchDescription([
        depth_to_laser,
        slam,
        # rtabmap_slam,
        # point_cloud_to_occupancy_grid,
    ])
