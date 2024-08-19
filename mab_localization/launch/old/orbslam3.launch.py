import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    orb_slam3_launch_arg = DeclareLaunchArgument(
        'orb_slam3_path',
        default_value=os.path.abspath(os.path.expanduser('~/ORB_SLAM3')),
        description='Path to the ORB_SLAM3 package'
    )

    mode = 'stereo-inertial'
    mode = 'rgbd'

    # Set the path to the shared library for ORB_SLAM3
    orb_slam3_path = LaunchConfiguration('orb_slam3_path')
    absolute_orb_slam3_path = PathJoinSubstitution([
        os.path.abspath(os.curdir), orb_slam3_path
    ])
    orb_slam3_lib_path = PathJoinSubstitution([
        absolute_orb_slam3_path, 'lib'
    ])

    # Get the current LD_LIBRARY_PATH or use an empty string if it doesn't exist
    current_ld_library_path = os.environ.get('LD_LIBRARY_PATH')

    # Set the LD_LIBRARY_PATH environment variable
    set_ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [orb_slam3_lib_path, TextSubstitution(text=':'), TextSubstitution(text=current_ld_library_path)]
    )

    # Load the vocabulary and settings files
    pkg_name = "mab_localization"
    pkg_share = get_package_share_directory(pkg_name)
    voc_file = os.path.join(pkg_share, 'resource', 'ORBvoc.txt')
    settings_file = os.path.join(pkg_share, 'resource', f'realsense_d435i_{mode}.yaml')
    # settings_file = '/home/aditya/ros2_ws/src/orbslam3/config/rgb-d/RealSense_d435i.yaml'
    # settings_file = '/home/aditya/ros2_ws/src/orbslam3/config/stereo/RealSense_d435i.yaml'
    
    # Launch the ORB_SLAM3 node
    if mode == 'rgbd':
        remappings = [
            ('/camera/rgb', '/d435i_camera/color/image_raw'),
            ('/camera/depth', '/d435i_camera/depth/image_raw'),
        ]
    elif mode == 'stereo':
        remappings = [
            ('/camera/left', '/d435i_camera/color/image_raw'),
            ('/camera/right', '/d435i_camera/color/image_raw'),
        ]
    elif mode == 'stereo-inertial':
        remappings = [
            ('/camera/left', '/d435i_camera/infra1/image_raw'),
            ('/camera/right', '/d435i_camera/infra1/image_raw'),
            ('/imu', '/imu/out')
        ]

    orb_slam3 = Node(
        package='orbslam3',
        executable=mode,
        name='orb_slam3',
        output='screen',
        parameters=[{
            'voc_file': voc_file,
            'settings_file': settings_file,
            'camera_frame': 'camera_image_link',
        }],
        remappings=remappings,
        arguments=[voc_file, settings_file]
    )

    odom_base_link_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_odom_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    camera_odom_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_odom_transform_publisher',
        arguments=['0.290755', '0.000053', '.068583', str(-math.pi/2), '0', '0', 'odom', 'camera_image_link']
    )

    return LaunchDescription([
        orb_slam3_launch_arg,
        set_ld_library_path,
        odom_base_link_transform_publisher,
        camera_odom_transform_publisher,
        orb_slam3,
    ])
