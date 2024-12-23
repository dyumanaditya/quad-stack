import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition



def generate_launch_description():
    spawn_bringup_pkg_dir = get_package_share_directory('quadstack_bringup')
    spawn_bringup_launch = os.path.join(spawn_bringup_pkg_dir, 'launch', 'spawn_robot.launch.py')

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

    spawn_bringup_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_bringup_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
        }.items()
    )

    stand_node = Node(
        package='quadstack_stand',
        executable='quadstack_stand',
        name='quadstack_stand',
        output='screen',
        parameters=[{'robot': LaunchConfiguration('robot')}]
    )

    # Stand
    # Timer to delay the start of mab_stand_node
    # Can only stand when the robot is spawned in gazebo
    delay_stand_node = TimerAction(
        period=5.0,
        actions=[stand_node]
    )
    
    # Conditional expressions to check what robot we're dealing with
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
    
    

    # Publish robot state (IMU + Joint states)
    mab_robot_state_publisher = Node(
        package='mab_utils',
        executable='publish_robot_state',
        name='publish_robot_state',
        output='screen',
        parameters=[{'use_sim_time': True}, {'robot': LaunchConfiguration('robot')}],
        condition=mab_robot_condition
    )
    
    unitree_robot_state_publisher = Node(
        package='unitree_utils',
        executable='publish_robot_state',
        name='publish_robot_state',
        output='screen',
        parameters=[{'use_sim_time': True}, {'robot': LaunchConfiguration('robot')}],
        condition=unitree_robot_condition
    )

    # Launch policy
    mab_policy_node = Node(
        package='mab_locomotion',
        executable='mab_locomotion',
        name='mab_locomotion',
        output='screen',
        parameters=[{'robot': LaunchConfiguration('robot')}, {'use_sim_time': True}],
        condition=mab_robot_condition
    )
    
    unitree_policy_node = Node(
        package='unitree_locomotion',
        executable='unitree_locomotion',
        name='unitree_locomotion',
        output='screen',
        parameters=[{'robot': LaunchConfiguration('robot')}, {'use_sim_time': True}],
        condition=unitree_robot_condition
    )

    mab_delay_policy_node = TimerAction(
        period=8.0,
        actions=[mab_policy_node]
    )
    
    unitree_delay_policy_node = TimerAction(
        period=8.0,
        actions=[unitree_policy_node]
    )

    # Launch keyboard teleop
    # Define the keyboard teleop node using ExecuteProcess
    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run quadstack_keyboard_teleop teleop_node'],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        robot_arg,
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        spawn_bringup_include_launch,
        mab_robot_state_publisher,
        unitree_robot_state_publisher,
        delay_stand_node,
        mab_delay_policy_node,
        unitree_delay_policy_node,
        teleop_node
    ])

if __name__ == '__main__':
    generate_launch_description()