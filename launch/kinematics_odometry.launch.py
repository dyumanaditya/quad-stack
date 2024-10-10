import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    urdf = DeclareLaunchArgument(
        'urdf',
        default_value='',
        description='The URDF file to use for kinematics odometry'
    )

    kinematics_odometry = Node(
        package='legged_kinematics_odometry',
        executable='kinematics_odometry',
        name='legged_kinematics_odometry',
        output='screen',
        parameters=[{'urdf': LaunchConfiguration('urdf'), 'use_sim_time': True}],
    )

    odom_publisher = Node(
        package='legged_kinematics_odometry',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )
    
    imu_gt_fd = Node(
        package='legged_kinematics_odometry',
        executable='imu_gt_fd',
        name='imu_gt_fd',
        output='screen',
    )

    return LaunchDescription([
        urdf,
        kinematics_odometry,
        imu_gt_fd,
        odom_publisher,
    ])
