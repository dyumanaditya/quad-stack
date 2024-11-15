import os
import rclpy
import time 

from rclpy.node import Node as nd
from rosgraph_msgs.msg import Clock

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os 

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    return LaunchDescription(
        [
        Node(
            package='hb40_control',
            executable='hb40_control',
            output='screen',
            arguments=[])
        ]) 
