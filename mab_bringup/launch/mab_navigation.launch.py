import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    mab_bringup_pkg_dir = get_package_share_directory('mab_bringup')
    mab_navigation_pkg_dir = get_package_share_directory('mab_navigation')
    mab_localization_pkg_dir = get_package_share_directory('mab_localization')

    mab_bringup_teleop_launch = os.path.join(mab_bringup_pkg_dir, 'launch', 'mab_teleop.launch.py')
    mab_navigation_launch = os.path.join(mab_navigation_pkg_dir, 'launch', 'mab_navigation.launch.py')
    mab_localization_launch = os.path.join(mab_localization_pkg_dir, 'launch', 'rtabmap_launch.py')
    
    mab_bringup_teleop_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_bringup_teleop_launch)
    )

    mab_navigation_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_navigation_launch)
    )
    
    mab_localization_include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mab_localization_launch)
    )

    delayed_localization_launch = TimerAction(
        period=15.0,
        actions=[mab_localization_include_launch]
    )
    
    return LaunchDescription([
        mab_bringup_teleop_include_launch,
        mab_navigation_include_launch,
        delayed_localization_launch,
    ])

if __name__ == '__main__':
    generate_launch_description()
