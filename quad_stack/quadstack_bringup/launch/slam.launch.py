import os
import math
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition, IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    quadstack_bringup_pkg_dir = get_package_share_directory('quadstack_bringup')
    quadstack_localization_pkg_dir = get_package_share_directory('quadstack_localization')
    quadstack_utils_pkg_dir = get_package_share_directory('quadstack_utils')

    quadstack_bringup_teleop_launch = os.path.join(quadstack_bringup_pkg_dir, 'launch', 'teleop.launch.py')
    quadstack_localization_vo_launch = os.path.join(quadstack_localization_pkg_dir, 'launch', 'visual_odometry.launch.py')
    localization_slam_launch = os.path.join(quadstack_localization_pkg_dir, 'launch', 'slam.launch.py')
    quadstack_utils_image_rotation_launch = os.path.join(quadstack_utils_pkg_dir, 'launch', 'rotate_image.launch.py')
    quadstack_utils_image_stabilizer_launch = os.path.join(quadstack_utils_pkg_dir, 'launch', 'camera_frame_stabilizer.launch.py')

    # Launch parameters
    rosbag_arg = DeclareLaunchArgument(
        'rosbag',
        default_value='false',
        description='Play a rosbag or load a simulation'
    )
    
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Rosbag of real robot, or simulation'
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', os.path.join(quadstack_bringup_pkg_dir, 'rviz', 'quadstack_slam.rviz')],
    )
    
    quadstack_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_bringup_teleop_launch),
        condition=UnlessCondition(LaunchConfiguration('rosbag')),
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
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
    
    quadstack_localization_slam_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_slam_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'rosbag': LaunchConfiguration('rosbag'),
            'real_robot': LaunchConfiguration('real_robot'),
            'use_vel_map_constraints': LaunchConfiguration('use_vel_map_constraints'),
            'use_laser_stabilization': LaunchConfiguration('use_laser_stabilization')
        }.items()
    )

    image_rotation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_utils_image_rotation_launch)
    )

    camera_frame_stabilizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(quadstack_utils_image_stabilizer_launch)
    )

    delayed_localization_vo_launch = TimerAction(
        period=12.0,
        actions=[quadstack_localization_vo_include_launch],
        condition=UnlessCondition(LaunchConfiguration('odom_gt'))
    )
    
    delayed_localization_slam_launch = TimerAction(
        period=15.0,
        actions=[quadstack_localization_slam_include_launch, rviz]
    )

    base_link_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_transform',
        output='screen',
        # arguments=['0', '0', '0', f'{math.pi/2}', '0', '0', 'base_link', 'd435i_camera_link'],
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'd435i_camera_link'],
        condition=IfCondition(LaunchConfiguration('rosbag'))
    )

    odom_gt = Node(
        package='quadstack_utils',
        executable='odom_gt',
        name='odom_gt',
        output='screen',
        condition=IfCondition(LaunchConfiguration('odom_gt'))
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
    
    
    return LaunchDescription([
        rosbag_arg,
        real_robot_arg,
        robot_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        world_arg,
        odom_gt_arg,
        use_kinematics_odom_arg,
        use_vel_map_constraints_arg,
        use_laser_stabilization_arg,
        # base_link_transform,
        quadstack_bringup_teleop_include_launch,
        delayed_localization_vo_launch,
        odom_gt,
        delayed_localization_slam_launch,
        image_rotation,
        silver_badger_real_robot_relay,
        go2_real_robot_relay,
        # camera_frame_stabilizer,
    ])

if __name__ == '__main__':
    generate_launch_description()
