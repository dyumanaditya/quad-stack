import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = "mab_localization"
    pkg_share = get_package_share_directory(pkg_name)
    settings_file = os.path.join(pkg_share, 'resource', 'rf2o.yaml')
    
    # Convert depth to laser scan
    depth_to_laser = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[settings_file],
        remappings=[
            ('depth', '/d435_camera/depth/image_raw'),
            ('depth_camera_info', '/d435_camera/depth/camera_info'),
            ('scan', '/scan')
        ]
    )

    # rf2o
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : True,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 100.0}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'resource', 'silver_badger_slam.rviz')],
    )

    odom_frame_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # odom_frame_pub2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'body', 'odom']
    # )

    return LaunchDescription([
        depth_to_laser,
        rf2o,
        rviz,
        odom_frame_pub,
        # odom_frame_pub2,
    ])
