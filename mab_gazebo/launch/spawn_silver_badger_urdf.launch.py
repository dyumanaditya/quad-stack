import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    package_name = 'mab_description'  # Change this to your actual package name
    pkg_share = get_package_share_directory(package_name)
    
    # Set the path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'silver_badger.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='urdf_file',
            default_value=urdf_file,
            description='Absolute path to robot urdf file'
        ),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH', 
            [os.path.join(pkg_share)]
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': robot_desc}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'silver_badger', '-file', LaunchConfiguration('urdf_file'), '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
