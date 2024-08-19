import os
import math
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    mab_bringup_pkg_dir = get_package_share_directory('mab_bringup')
    mab_localization_pkg_dir = get_package_share_directory('mab_localization')
    mab_utils_pkg_dir = get_package_share_directory('mab_utils')

    mab_bringup_teleop_launch = os.path.join(mab_bringup_pkg_dir, 'launch', 'mab_teleop.launch.py')
    mab_localization_vo_launch = os.path.join(mab_localization_pkg_dir, 'launch', 'visual_odometry.launch.py')
    mab_localization_slam_launch = os.path.join(mab_localization_pkg_dir, 'launch', 'slam.launch.py')
    mab_utils_image_rotation_launch = os.path.join(mab_utils_pkg_dir, 'launch', 'rotate_image.launch.py')

    # Launch parameters
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mab_localization_pkg_dir, 'resource', 'map.yaml'),
        description='Play a rosbag or load a simulation'
    )

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger or honey_badger'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', os.path.join(mab_bringup_pkg_dir, 'rviz', 'mab_slam.rviz')],
    )
    
    mab_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_bringup_teleop_launch),
        condition=UnlessCondition(LaunchConfiguration('rosbag')),
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )
    
    mab_localization_vo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_localization_vo_launch)
    )
    
    mab_localization_slam_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_localization_slam_launch)
    )

    image_rotation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_utils_image_rotation_launch)
    )

    delayed_localization_vo_launch = TimerAction(
        period=12.0,
        actions=[mab_localization_vo_include_launch]
    )
    
    delayed_localization_slam_launch = TimerAction(
        period=10.0,
        actions=[mab_localization_slam_include_launch, rviz]
    )
    
    return LaunchDescription([
        map_arg,
        robot_arg,
        mab_bringup_teleop_include_launch,
        delayed_localization_vo_launch,
        delayed_localization_slam_launch,
        image_rotation,
    ])

if __name__ == '__main__':
    generate_launch_description()
