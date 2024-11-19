import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals



def generate_launch_description():
    mab_bringup_pkg_dir = get_package_share_directory('mab_bringup')
    mab_navigation_pkg_dir = get_package_share_directory('mab_navigation')
    mab_localization_pkg_dir = get_package_share_directory('mab_localization')
    mab_utils_pkg_dir = get_package_share_directory('mab_utils')

    mab_bringup_teleop_launch = os.path.join(mab_bringup_pkg_dir, 'launch', 'mab_teleop.launch.py')
    mab_navigation_launch = os.path.join(mab_navigation_pkg_dir, 'launch', 'navigation.launch.py')
    mab_localization_vo_launch = os.path.join(mab_localization_pkg_dir, 'launch', 'visual_odometry.launch.py')
    mab_localization_slam_launch = os.path.join(mab_localization_pkg_dir, 'launch', 'slam.launch.py')
    mab_utils_image_rotation_launch = os.path.join(mab_utils_pkg_dir, 'launch', 'rotate_image.launch.py')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Load a pre-saved map'
    )
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger or honey_badger'
    )

    mab_localization_slam_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_localization_slam_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items()
    )

    mab_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_bringup_teleop_launch),
        # condition=LaunchConfigurationEquals('map', '')
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )

    mab_navigation_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_navigation_launch)
    )
    
    delayed_navigation_launch = TimerAction(
        period=5.0,
        actions=[mab_navigation_include_launch]
    )

    mab_localization_vo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_localization_vo_launch)
    )

    image_rotation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_utils_image_rotation_launch)
    )
    
    delayed_localization_vo_launch = TimerAction(
        period=12.0,
        actions=[mab_localization_vo_include_launch]
    )

    delayed_localization_slam_launch = TimerAction(
        period=12.0,
        actions=[mab_localization_slam_include_launch]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', os.path.join(mab_bringup_pkg_dir, 'rviz', 'mab_nav.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    map_odom_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_odom_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Only if map is not provided, we launch the slam. Otherwise we do navigation on the received map
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        robot_arg,

        mab_bringup_teleop_include_launch,
        delayed_navigation_launch,
        delayed_localization_vo_launch,
        image_rotation,
        rviz,

        GroupAction(
            actions=[
                map_odom_transform_publisher,
            ],
            condition=LaunchConfigurationNotEquals('map', '')
        ),
        GroupAction(
            actions=[
                delayed_localization_slam_launch,
            ],
            condition=LaunchConfigurationEquals('map', '')
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
