import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    # Get the package directory
    use_ros_control = False
    description_package_name = 'mab_description'
    gazebo_package_name = 'mab_gazebo'
    plugin_package_name = 'mab_control'
    realsense_package_name = 'realsense_gazebo_plugin'
    description_pkg_share = get_package_share_directory(description_package_name)
    gazebo_pkg_share = get_package_share_directory(gazebo_package_name)
    gazebo_pkg_prefix = get_package_prefix(plugin_package_name)
    realsense_pkg_prefix = get_package_prefix(realsense_package_name)

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger or honey_badger'
    )
    
    # Set the path to the Xacro file
    silver_badger_xacro_file = os.path.join(description_pkg_share, 'xacro', 'silver_badger.urdf_new_inertia.xacro')
    # silver_badger_xacro_file = os.path.join(description_pkg_share, 'xacro', 'silver_badger.urdf.xacro')
    honey_badger_xacro_file = os.path.join(description_pkg_share, 'xacro', 'honey_badger.urdf.xacro')

    # Launch configuration variables specific to simulation
    # x_pose = LaunchConfiguration('x_pose', default='0')
    # y_pose = LaunchConfiguration('y_pose', default='2')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='3.5')

    # Controller configuration
    controller_config = os.path.join(gazebo_pkg_share, 'config', 'silver_badger_controller.yaml')

    gazebo_env_variable = SetEnvironmentVariable('GAZEBO_MODEL_PATH', [os.path.join(description_pkg_share)])
    os.environ["GAZEBO_MODEL_PATH"] = description_pkg_share 
    # os.environ["GAZEBO_MODEL_PATH"] = get_package_share_directory('turtlebot3_description_custom') + ':' + os.environ["GAZEBO_MODEL_PATH"] 
    gazebo_plugin_path = os.path.join(gazebo_pkg_prefix, 'lib', 'mab_control')
    realsense_plugin_path = os.path.join(realsense_pkg_prefix, 'lib')


    plugin_paths = gazebo_plugin_path + ':' + realsense_plugin_path
    os.environ['GAZEBO_PLUGIN_PATH'] = plugin_paths

    # Launch Gazebo with a world file
    world_pkg = 'turtlebot3_gazebo'
    # world_file = os.path.join(gazebo_pkg_share, 'worlds', 'mab_house.world')
    # world_file = os.path.join(gazebo_pkg_share, 'worlds', 'mab_house_tires.world')
    world_file = os.path.join(gazebo_pkg_share, 'worlds', 'empty_world.world')
    gazebo_models_path = os.path.join(get_package_share_directory(world_pkg), 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ':' + os.environ["GAZEBO_MODEL_PATH"]
    gazebo = ExecuteProcess(
        # cmd=['gazebo', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        # cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
        # cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gazebo_ros_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mab_quadruped', '-topic', '/robot_description', '-x', x_pose, '-y', y_pose, '-z', '0.1'],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher_silver_badger = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', silver_badger_xacro_file]), value_type=str),
        }],
        condition=LaunchConfigurationEquals('robot', 'silver_badger')
    )
    
    robot_state_publisher_honey_badger = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':True,
            'robot_description': ParameterValue(Command(['xacro ', honey_badger_xacro_file]), value_type=str),
        }],
        condition=LaunchConfigurationEquals('robot', 'honey_badger')
    )

    if use_ros_control:
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

        args = [
            # use_sim_time,
            gazebo,
            gazebo_ros_robot,
            robot_state_publisher_silver_badger,
            joint_state_broadcaster,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
        ]

    else:
        args = [
            gazebo,
            robot_arg,
            gazebo_ros_robot,
            robot_state_publisher_silver_badger,
            robot_state_publisher_honey_badger,
            # map_frame_pub,
            # joint_state_publisher
        ]

    return LaunchDescription(args)



if __name__ == '__main__':
    generate_launch_description()
