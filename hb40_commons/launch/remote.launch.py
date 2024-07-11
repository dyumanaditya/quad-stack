import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    robot_prefix = 'hb40/'

    joy_node = Node(
            package='joy',
            executable='joy_node',
            output='screen',
            remappings=[
                ('/joy', robot_prefix + 'joy')],
            parameters=[
                {'coalesce_interval': 0.1,
                'autorepeat_rate': 10.0}
            ]
    )

    teleop_node = Node(
            package='hb40_remote',
            executable='hb40_teleop',
            output='screen',
            remappings=[]
    )

    remote_node = Node(
            package='hb40_remote',
            executable='hb40_remote',
            output='screen',
            parameters=[],  
            arguments=[],
            remappings=[])

    return LaunchDescription(
        [
            remote_node,
            joy_node,
            teleop_node
        ]) 
