import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_name = "quadstack_utils"
    pkg_share = get_package_share_directory(pkg_name)
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
    )
    
    rosbag_arg = DeclareLaunchArgument(
        'rosbag',
        default_value='false',
        description='Play a rosbag or load a simulation'
    )
    
    laser_params = PythonExpression([
        "'laser_real.yaml' if '", LaunchConfiguration('real_robot'), "' == 'true' else 'laser_sim.yaml'"
    ])
    laser_settings_file = [os.path.join(pkg_share, 'resource', ''), laser_params]
    
    mab_robot_condition = IfCondition(
        PythonExpression(
            [
                '"', LaunchConfiguration('robot'), '" == "silver_badger" or "',
                LaunchConfiguration('robot'), '" == "honey_badger"'
            ]
        )
    )
    
    unitree_robot_condition = IfCondition(
        PythonExpression(
            [
                '"', LaunchConfiguration('robot'), '" == "a1" or "',
                LaunchConfiguration('robot'), '" == "go1" or "',
                LaunchConfiguration('robot'), '" == "go2"'
            ]
        )
    )
    
    # Whether to use stabilization or not
    laser_stabilization = LaunchConfiguration('use_laser_stabilization')
    use_laser_stabilization = PythonExpression([
        "True if '", laser_stabilization, "' == 'true' else False"
    ])

    depth_to_laser_mab = Node(
        package='depthimage_to_laserscan_stabilized',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            {'use_imu_stabilization': use_laser_stabilization},
            laser_settings_file
        ],
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
        ],
        condition=mab_robot_condition
    )
    
    depth_to_laser_unitree = Node(
        package='depthimage_to_laserscan_stabilized',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            {'use_imu_stabilization': use_laser_stabilization},
            laser_settings_file
        ],
        remappings=[
            # ('depth', '/d435i_camera/depth/image_rect_raw/rotated'),
            # ('depth_camera_info', '/d435i_camera/depth/image_rect_raw/rotated'),
            ('depth', '/d435i_camera/depth/image_rect_raw'),
            ('depth_camera_info', '/d435i_camera/depth/camera_info'),
            # ('/depth', '/d435i_camera/depth/image_rect_raw/stabilized'),
            # ('/depth_camera_info', '/d435i_camera/depth/camera_info/stabilized'),
            # ('depth', '/waffle_cam/depth/image_raw'),
            # ('depth_camera_info', '/waffle_cam/depth/camera_info'),
            ('scan', '/scan'),
            ('imu', '/imu/out'),
        ],
        condition=unitree_robot_condition
    )

    project_laser_frame = Node(
        package='quadstack_utils',
        executable='project_laser_frame',
        name='project_laser_frame',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        # project_laser_frame,
        robot_arg,
        rosbag_arg,
        depth_to_laser_mab,
        depth_to_laser_unitree,
    ])
