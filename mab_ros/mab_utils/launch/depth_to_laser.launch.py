import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    pkg_name = "mab_utils"
    pkg_share = get_package_share_directory(pkg_name)
    laser_settings_file = os.path.join(pkg_share, 'resource', 'laser.yaml')

    depth_to_laser = Node(
        package='depthimage_to_laserscan_stabilized',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[laser_settings_file],
        remappings=[
            ('depth', '/d435i_camera/depth/image_rect_raw/rotated'),
            ('depth_camera_info', '/d435i_camera/depth/camera_info/rotated'),
            # ('depth', '/d435i_camera/depth/image_raw'),
            # ('depth_camera_info', '/d435i_camera/depth/camera_info'),
            # ('/depth', '/d435i_camera/depth/image_rect_raw/stabilized'),
            # ('/depth_camera_info', '/d435i_camera/depth/camera_info/stabilized'),
            # ('depth', '/waffle_cam/depth/image_raw'),
            # ('depth_camera_info', '/waffle_cam/depth/camera_info'),
            ('scan', '/scan'),
            ('imu', '/imu/out'),
        ]
    )

    project_laser_frame = Node(
        package='mab_utils',
        executable='project_laser_frame',
        name='project_laser_frame',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        # project_laser_frame,
        depth_to_laser,
    ])
