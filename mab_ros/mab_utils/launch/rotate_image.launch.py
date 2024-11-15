from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rotate_image = Node(
        package='mab_utils',
        executable='image_rotate',
        name='image_rotate_node',
        output='screen',
        remappings=[
            ('/input_depth_image', '/d435i_camera/depth/image_rect_raw'),
            ('/input_depth_camera_info', '/d435i_camera/depth/camera_info'),
            ('/input_rgb_image', '/d435i_camera/color/image_raw'),
            ('/input_rgb_camera_info', '/d435i_camera/color/camera_info'),
            ('/rotated_depth_image', '/d435i_camera/depth/image_rect_raw/rotated'),
            ('/rotated_depth_camera_info', '/d435i_camera/depth/camera_info/rotated'),
            ('/rotated_rgb_image', '/d435i_camera/color/image_raw/rotated'),
            ('/rotated_rgb_camera_info', '/d435i_camera/color/camera_info/rotated'),
        ],
    )

    return LaunchDescription([
        rotate_image,
    ])
