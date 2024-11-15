import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import json 

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    config_path = os.path.expanduser("~") + "/.ros/mab/config/global.json"
    with open(config_path, "r") as f:
        config = json.load(f)
    robot_type = config["robot_type"]
    robot_prefix = config["robot_name"] + "/"
    urdf_file_name = robot_type + '.urdf'
    urdf = os.path.join(
        get_package_share_directory('hb40_commons'), "models", "urdf",
        urdf_file_name)
    with open(urdf, 'r') as file:
        robot_desc = file.read()

    bridge_node = Node(
            package='hb40_bridge',
            executable='hb40_bridge',
            output='screen',
    )

    ros2_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mab_robot_state_publisher_node',
            output='screen',
            parameters=[{ 'robot_description': robot_desc,
                            'publish_frequency': 10.0,
                            'frame_prefix': robot_prefix
                            }],  
            arguments=[],
            remappings=[
                ('/joint_states', robot_prefix + 'joint_states'),
                ('/robot_description', robot_prefix + 'robot_description'),
                ])

    return LaunchDescription(
        [
            ros2_robot_state_publisher,
            bridge_node
        ]) 
