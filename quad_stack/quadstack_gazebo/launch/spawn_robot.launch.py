import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals



def generate_launch_description():
    # Get the package directory
    gazebo_package_name = 'quadstack_gazebo'
    realsense_package_name = 'realsense_gazebo_plugin'
    gazebo_pkg_share = get_package_share_directory(gazebo_package_name)
    realsense_pkg_prefix = get_package_prefix(realsense_package_name)
    
    sb_description_pkg_share = get_package_share_directory('silver_badger_description')
    hb_description_pkg_share = get_package_share_directory('honey_badger_description')
    a1_description_pkg_share = get_package_share_directory('a1_description')
    go1_description_pkg_share = get_package_share_directory('go1_description')
    go2_description_pkg_share = get_package_share_directory('go2_description')
    
    description_pkg_share = {
        'silver_badger': sb_description_pkg_share,
        'honey_badger': hb_description_pkg_share,
        'a1': a1_description_pkg_share,
        'go1': go1_description_pkg_share,
        'go2': go2_description_pkg_share,
    }
    
    # Based on the robot selected, set the robot description file
    urdf_paths = {
        'silver_badger': os.path.join(sb_description_pkg_share, 'urdf', 'silver_badger.urdf'),
        'honey_badger': os.path.join(hb_description_pkg_share, 'urdf', 'honey_badger.urdf'),
        'a1': os.path.join(a1_description_pkg_share, 'urdf', 'a1.urdf'),
        'go1': os.path.join(go1_description_pkg_share, 'urdf', 'go1.urdf'),
        'go2': os.path.join(go2_description_pkg_share, 'urdf', 'go2.urdf'),
    }
    
    robot_descriptions = {}
    for robot, path in urdf_paths.items():
        with open(path, 'r') as file:
            # Read the file and replace the package path with the package share directory (package:// doesn't work in Gazebo)
            robot_descriptions[robot] = file.read()
            robot_descriptions[robot] = robot_descriptions[robot].replace(f'package://{robot}_description', description_pkg_share[robot])
            robot_descriptions[robot] = robot_descriptions[robot].replace('package://realsense2_description', get_package_share_directory('realsense2_description'))

    # Launch configuration variables specific to simulation
    robot = LaunchConfiguration('robot', default='silver_badger')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='3.5')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # gazebo_env_variable = SetEnvironmentVariable('GAZEBO_MODEL_PATH', sb_description_pkg_share)
    # os.environ["GAZEBO_RESOURCE_PATH"] = '$GAZEBO_RESOURCE_PATH:' + sb_description_pkg_share
    # os.environ["GAZEBO_MODEL_PATH"] = sb_description_pkg_share + ':' + hb_description_pkg_share + ':' + a1_description_pkg_share + ':' + go1_description_pkg_share + ':' + go2_description_pkg_share
    realsense_plugin_path = os.path.join(realsense_pkg_prefix, 'lib')
    gazebo_plugin_path = os.path.join(get_package_prefix('mab_control'), 'lib', 'mab_control')


    plugin_paths = gazebo_plugin_path + ':' + realsense_plugin_path
    os.environ['GAZEBO_PLUGIN_PATH'] = plugin_paths

    # Launch Gazebo with a world file
    world_pkg = 'turtlebot3_gazebo'
    # world_file = os.path.join(gazebo_pkg_share, 'worlds', 'mab_house.world')
    # world_file = os.path.join(gazebo_pkg_share, 'worlds', 'mab_house_tires.world')
    world_file = os.path.join(gazebo_pkg_share, 'worlds', 'empty_world.world')
    gazebo_models_path = os.path.join(get_package_share_directory(world_pkg), 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ':' + os.environ["GAZEBO_MODEL_PATH"]
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    gazebo = ExecuteProcess(
        # cmd=['gazebo', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        # cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
        # cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gazebo_ros_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # arguments=['-entity', robot, '-file', urdf_paths['silver_badger'], '-x', x_pose, '-y', y_pose, '-z', z_pose],
        arguments=['-entity', robot, '-topic', '/robot_description', '-x', x_pose, '-y', y_pose, '-z', z_pose],
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
            'robot_description': robot_descriptions['silver_badger'],
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
            'robot_description': robot_descriptions['honey_badger'],
        }],
        condition=LaunchConfigurationEquals('robot', 'honey_badger')
    )
    
    robot_state_publisher_a1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':True,
            'robot_description': robot_descriptions['a1'],
        }],
        condition=LaunchConfigurationEquals('robot', 'a1')
    )
    
    robot_state_publisher_go1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':True,
            'robot_description': robot_descriptions['go1'],
        }],
        condition=LaunchConfigurationEquals('robot', 'go1')
    )
    
    robot_state_publisher_go2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':True,
            'robot_description': robot_descriptions['go2'],
        }],
        condition=LaunchConfigurationEquals('robot', 'go2')
    )
    
    


    args = [
        gazebo,
        gazebo_ros_robot,
        # gazebo_env_variable,

        robot_state_publisher_silver_badger,
        robot_state_publisher_honey_badger,
        robot_state_publisher_a1,
        robot_state_publisher_go1,
        robot_state_publisher_go2,
        # map_frame_pub,
        # joint_state_publisher
    ]

    return LaunchDescription(args)



if __name__ == '__main__':
    generate_launch_description()
