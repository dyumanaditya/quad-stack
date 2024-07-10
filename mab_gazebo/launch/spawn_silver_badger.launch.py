import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directory
    description_package_name = 'mab_description'
    gazebo_package_name = 'mab_gazebo'
    description_pkg_share = get_package_share_directory(description_package_name)
    gazebo_pkg_share = get_package_share_directory(gazebo_package_name)
    
    # Set the path to the Xacro file
    xacro_file = os.path.join(description_pkg_share, 'xacro', 'silver_badger.urdf.xacro')
    urdf_file = os.path.join(description_pkg_share, 'urdf', 'silver_badger.urdf')

    with open(urdf_file, 'r') as infp:
        urdf = infp.read()

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Controller configuration
    controller_config = os.path.join(gazebo_pkg_share, 'config', 'silver_badger_controller.yaml')

    # Declare the launch options
    use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    gazebo_env_variable = SetEnvironmentVariable('GAZEBO_MODEL_PATH', [os.path.join(description_pkg_share)])
    gazebo = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
        # cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gazebo_ros_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'silver_badger', '-topic', '/robot_description', '-x', x_pose, '-y', y_pose, '-z', '0.0'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'), 
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
            # 'robot_description': urdf
        }],
        # arguments=[urdf_file]
    )

    # Not used -- happens in gazebo xacro file
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[controller_config]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller],
        )
    )

    return LaunchDescription([
        use_sim_time,
        gazebo,
        gazebo_ros_robot,
        robot_state_publisher,
        joint_state_broadcaster,
        # joint_trajectory_controller,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])

if __name__ == '__main__':
    generate_launch_description()
