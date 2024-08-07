import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_name = "mab_navigation"
    pkg_share = get_package_share_directory(pkg_name)
    nav2_settings_file = os.path.join(pkg_share, 'resource', 'nav2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # 'params_file': nav2_settings_file,
                # 'autostart': True,
                # 'map': '/map',
            }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2,
    ])
