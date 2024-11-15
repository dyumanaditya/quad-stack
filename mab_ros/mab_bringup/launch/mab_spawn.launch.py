import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    mab_gazebo_pkg_dir = get_package_share_directory('mab_gazebo')
    mab_gazebo_launch = os.path.join(mab_gazebo_pkg_dir, 'launch', 'spawn_robot.launch.py')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger or honey_badger'
    )

    mab_gazebo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_gazebo_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot')
        }.items()
    )

    
    return LaunchDescription([
        robot_arg,
        mab_gazebo_include_launch,
    ])



if __name__ == '__main__':
    generate_launch_description()