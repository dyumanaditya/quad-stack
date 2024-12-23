import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    gazebo_pkg_dir = get_package_share_directory('quadstack_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg_dir, 'launch', 'spawn_robot.launch.py')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mab_house_tires.world',
        description='Specify the Gazebo world file to load'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.0',
        description='X position of the robot at start'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='3.5',
        description='Y position of the robot at start'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of the robot at start'
    )

    gazebo_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
        }.items()
    )

    
    return LaunchDescription([
        robot_arg,
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        gazebo_include_launch,
    ])



if __name__ == '__main__':
    generate_launch_description()