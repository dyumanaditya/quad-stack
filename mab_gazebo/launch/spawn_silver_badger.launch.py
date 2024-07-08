import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directory
    package_name = 'mab_description'  # Change this to your actual package name
    pkg_share = get_package_share_directory(package_name)
    
    # Set the path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'silver_badger.urdf.xacro')

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # SetEnvironmentVariable(
        #     'GAZEBO_MODEL_PATH', 
        #     [os.path.join(pkg_share)]
        # ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'), 
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'silver_badger', '-topic', '/robot_description', '-x', x_pose, '-y', y_pose, '-z', '0.0'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
