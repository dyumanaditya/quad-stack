from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'subscribe_rgbd': True,
                'frame_id': 'camera_image_link',
                'queue_size': 100,
                # 'topic_queue_size': 100,
            }],
            remappings=[
                ('rgb/image', '/d435_camera/color/image_raw'),
                ('depth/image', '/d435_camera/depth/image_raw'),
                ('rgb/camera_info', '/d435_camera/color/camera_info'),
                ('rgbd_image', '/rgbd_image')
            ],
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_image_link',
                'approx_sync': True,
                # 'subscribe_rgbd': True,
                # 'odom_frame_id': 'odom'
            }],
            # arguments=['--ros-args', '--log-level', 'DEBUG']
            remappings=[
                # ('/rgb/image', '/d435_camera/color/image_raw'),
                # ('/depth/image', '/d435_camera/depth/image_raw'),
                # ('/rgb/camera_info', '/d435_camera/color/camera_info')
            ]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_image_link', 'base_link']
        # )
    ])
