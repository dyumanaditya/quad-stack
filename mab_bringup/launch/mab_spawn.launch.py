import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    mab_gazebo_pkg_dir = get_package_share_directory('mab_gazebo')
    mab_gazebo_launch = os.path.join(mab_gazebo_pkg_dir, 'launch', 'spawn_silver_badger.launch.py')
    mab_gazebo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_gazebo_launch)
    )

    
    return LaunchDescription([
        mab_gazebo_include_launch,
    ])



if __name__ == '__main__':
    generate_launch_description()