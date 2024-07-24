import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    mab_gazebo_pkg_dir = get_package_share_directory('mab_gazebo')
    mab_gazebo_launch = os.path.join(mab_gazebo_pkg_dir, 'launch', 'spawn_silver_badger.launch.py')
    mab_gazebo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_gazebo_launch)
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
        output='screen'
    )

    # Launch policy
    policy_node = Node(
        package='mab_locomotion',
        executable='mab_locomotion',
        name='mab_locomotion',
        output='screen'
    )

    # Launch keyboard teleop
    # Define the keyboard teleop node using ExecuteProcess
    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run mab_keyboard_teleop teleop_node'],
        output='screen',
        shell=True
    )


    return LaunchDescription([
        mab_gazebo_include_launch,
        robot_state_publisher,
        policy_node,
        delay_mab_stand_node,
        teleop_node
    ])



if __name__ == '__main__':
    generate_launch_description()