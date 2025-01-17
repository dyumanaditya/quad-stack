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
    quadstack_bringup_pkg_dir = get_package_share_directory('quadstack_bringup')
    quadstack_navigation_pkg_dir = get_package_share_directory('quadstack_navigation')
    quadstack_localization_pkg_dir = get_package_share_directory('quadstack_localization')
    quadstack_utils_pkg_dir = get_package_share_directory('quadstack_utils')

    quadstack_bringup_teleop_launch = os.path.join(quadstack_bringup_pkg_dir, 'launch', 'teleop.launch.py')
    quadstack_navigation_launch = os.path.join(quadstack_navigation_pkg_dir, 'launch', 'navigation.launch.py')
    quadstack_localization_vo_launch = os.path.join(quadstack_localization_pkg_dir, 'launch', 'visual_odometry.launch.py')
    quadstack_localization_slam_launch = os.path.join(quadstack_localization_pkg_dir, 'launch', 'slam.launch.py')
    quadstack_utils_image_rotation_launch = os.path.join(quadstack_utils_pkg_dir, 'launch', 'rotate_image.launch.py')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Load a pre-saved map'
    )
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mab_house_tires.world',
        description='Specify the Gazebo world file to load'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.0',
        description='X position of the robot at start'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='3.5',
        description='Y position of the robot at start'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of the robot at start'
    )
    
    odom_gt_arg = DeclareLaunchArgument(
        'odom_gt',
        default_value='false',
        description='Whether to use ground truth odometry'
    )

    quadstack_localization_slam_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_localization_slam_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items()
    )

    mab_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_bringup_teleop_launch),
        # condition=LaunchConfigurationEquals('map', '')
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )

    mab_navigation_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_navigation_launch)
    )
    
    delayed_navigation_launch = TimerAction(
        period=5.0,
        actions=[mab_navigation_include_launch]
    )

    mab_localization_vo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_localization_vo_launch)
    )

    image_rotation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_utils_image_rotation_launch)
    )
    
    delayed_localization_vo_launch = TimerAction(
        period=12.0,
        actions=[mab_localization_vo_include_launch]
    )

    delayed_localization_slam_launch = TimerAction(
        period=15.0,
        actions=[quadstack_localization_slam_include_launch]
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
        arguments=['-d', os.path.join(quadstack_bringup_pkg_dir, 'rviz', 'quadstack_nav.rviz')],
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
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        odom_gt_arg,

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
