import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    mab_bringup_pkg_dir = get_package_share_directory('mab_bringup')
    mab_bringup_launch = os.path.join(mab_bringup_pkg_dir, 'launch', 'mab_spawn.launch.py')

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger or honey_badger'
    )

    mab_bringup_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_bringup_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )

    mab_stand_node = Node(
        package='mab_stand',
        executable='mab_stand',
        name='mab_stand',
        output='screen'
    )

    # Stand
    # Timer to delay the start of mab_stand_node
    # Can only stand when the robot is spawned in gazebo
    delay_mab_stand_node = TimerAction(
        period=5.0,
        actions=[mab_stand_node]
    )

    # Publish robot state (IMU + Joint states)
    robot_state_publisher = Node(
        package='mab_utils',
        executable='publish_robot_state',
        name='publish_robot_state',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch policy
    policy_node = Node(
        package='mab_locomotion',
        executable='mab_locomotion',
        name='mab_locomotion',
        output='screen',
        parameters=[{'robot': LaunchConfiguration('robot')}, {'use_sim_time': True}]
    )

    delay_policy_node = TimerAction(
        period=8.0,
        actions=[policy_node]
    )

    # Launch keyboard teleop
    # Define the keyboard teleop node using ExecuteProcess
    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run mab_keyboard_teleop teleop_node'],
        output='screen',
        shell=True
    )


    return LaunchDescription([
        robot_arg,
        mab_bringup_include_launch,
        robot_state_publisher,
        delay_mab_stand_node,
        delay_policy_node,
        teleop_node
    ])



if __name__ == '__main__':
    generate_launch_description()