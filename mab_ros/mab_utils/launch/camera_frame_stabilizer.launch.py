from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_frame_stabilizer = Node(
        package='mab_utils',
        executable='camera_frame_stabilizer',
        name='camera_frame_stabilizer',
        output='screen',
        remappings=[
            ('/depth_input', '/d435i_camera/depth/image_rect_raw/rotated'),
            ('/depth_stabilized', '/d435i_camera/depth/image_rect_raw/stabilized'),
            ('/depth_camera_info', '/d435i_camera/depth/camera_info/rotated'),
            ('/depth_stabilized_camera_info', '/d435i_camera/depth/camera_info/stabilized'),
            # ('/depth_input', '/d435i_camera/depth/image_rect_raw'),
            # ('/depth_stabilized', '/d435i_camera/depth/image_rect_raw/stabilized'),
            # ('/depth_camera_info', '/d435i_camera/depth/camera_info'),
            # ('/depth_stabilized_camera_info', '/d435i_camera/depth/camera_info/stabilized'),
            # ('/imu/out', '/imu'),
        ]
    )

    return LaunchDescription([
        camera_frame_stabilizer,
    ])
