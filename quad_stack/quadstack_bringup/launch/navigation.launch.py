import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationNotEquals, IfCondition, LaunchConfigurationEquals



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

    rosbag_arg = DeclareLaunchArgument(
        'rosbag',
        default_value='false',
        description='Play a rosbag or load a simulation'
    )
    
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
    
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Rosbag of real robot, or simulation'
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
    
    use_kinematics_odom_arg = DeclareLaunchArgument(
        'use_kinematics_odom',
        default_value='true',
        description='Use kinematics odometry'
    )
    
    use_vel_map_constraints_arg = DeclareLaunchArgument(
        'use_vel_map_constraints',
        default_value='true',
        description='Add velocity constraints to the map'
    )
    
    use_laser_stabilization_arg = DeclareLaunchArgument(
        'use_laser_stabilization',
        default_value='true',
        description='Whether to stabilize the laser scan based on IMU data'
    )

    quadstack_localization_slam_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_localization_slam_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'rosbag': LaunchConfiguration('rosbag'),
            'real_robot': LaunchConfiguration('real_robot'),
            'use_vel_map_constraints': LaunchConfiguration('use_vel_map_constraints'),
            'use_laser_stabilization': LaunchConfiguration('use_laser_stabilization')
        }.items()
    )

    quadstack_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_bringup_teleop_launch),
        # condition=LaunchConfigurationEquals('map', '')
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )

    quadstack_navigation_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_navigation_launch)
    )
    
    delayed_navigation_launch = TimerAction(
        period=5.0,
        actions=[quadstack_navigation_include_launch]
    )

    quadstack_localization_vo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_localization_vo_launch),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'robot': LaunchConfiguration('robot'),
            'use_kinematics_odom': LaunchConfiguration('use_kinematics_odom'),
            'rosbag': LaunchConfiguration('rosbag'),
            'real_robot': LaunchConfiguration('real_robot')
        }.items()
    )

    image_rotation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_utils_image_rotation_launch)
    )
    
    delayed_localization_vo_launch = TimerAction(
        period=12.0,
        actions=[quadstack_localization_vo_include_launch]
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
    
    sb_real_rosbag_condition = IfCondition(
        PythonExpression(
            [
                '"', LaunchConfiguration('robot'), '" == "silver_badger" and "',
                LaunchConfiguration('rosbag'), '" == "true" and "',
                LaunchConfiguration('real_robot'), '" == "true"'
                
            ]
        )
    )
    
    go2_real_rosbag_condition = IfCondition(
        PythonExpression(
            [
                '"', LaunchConfiguration('robot'), '" == "go2" and "',
                LaunchConfiguration('rosbag'), '" == "true" and "',
                LaunchConfiguration('real_robot'), '" == "true"'
            ]
        )
    )
    
    silver_badger_real_robot_relay = Node(
        package='mab_utils',
        executable='silver_badger_real_robot_relay',
        name='silver_badger_real_robot_relay',
        output='screen',
        condition=sb_real_rosbag_condition
    )
    
    go2_real_robot_relay = Node(
        package='unitree_utils',
        executable='go2_real_robot_relay',
        name='go2_real_robot_relay',
        output='screen',
        condition=go2_real_rosbag_condition
    )
    
    # Only if map is not provided, we launch the slam. Otherwise we do navigation on the received map
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        rosbag_arg,
        robot_arg,
        world_arg,
        real_robot_arg,
        use_kinematics_odom_arg,
        use_vel_map_constraints_arg,
        use_laser_stabilization_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        odom_gt_arg,

        silver_badger_real_robot_relay,
        go2_real_robot_relay,

        quadstack_bringup_teleop_include_launch,
        delayed_navigation_launch,
        delayed_localization_vo_launch,
        image_rotation,
        rviz,

        # GroupAction(
        #     actions=[
        #         map_odom_transform_publisher,
        #     ],
        #     condition=LaunchConfigurationNotEquals('map', '')
        # ),
        GroupAction(
            actions=[
                delayed_localization_slam_launch,
            ],
            condition=LaunchConfigurationEquals('map', '')
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
