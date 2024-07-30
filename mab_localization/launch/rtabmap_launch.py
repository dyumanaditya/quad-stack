import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_name = "mab_localization"
    pkg_share = get_package_share_directory(pkg_name)
    laser_settings_file = os.path.join(pkg_share, 'resource', 'laser.yaml')
    slam_settings_file = os.path.join(pkg_share, 'resource', 'slam_toolbox_params.yaml')

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
            'publish_tf': True,
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG']
        remappings=[
            ('/rgb/image', '/d435_camera/color/image_raw'),
            ('/depth/image', '/d435_camera/depth/image_raw'),
            ('/rgb/camera_info', '/d435_camera/color/camera_info')
        ]
    )

    depth_to_laser = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[laser_settings_file],
        remappings=[
            ('depth', '/d435_camera/depth/image_raw'),
            ('depth_camera_info', '/d435_camera/depth/camera_info'),
            ('scan', '/scan')
        ]
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


    return LaunchDescription([
        rviz,
        rtabmap,
        depth_to_laser,
        slam
    ])
