import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationNotEquals



def generate_launch_description():
    pkg_name = "quadstack_navigation"
    pkg_share = get_package_share_directory(pkg_name)
    amcl_settings_file = os.path.join(pkg_share, 'resource', 'amcl.yaml')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
    )
    
    nav2_mab_file = os.path.join(pkg_share, 'resource', 'nav2_mab.yaml')
    nav2_unitree_file = os.path.join(pkg_share, 'resource', 'nav2_unitree.yaml')

    nav2_settings_file = PythonExpression([
        f"'{nav2_mab_file}' if '", LaunchConfiguration('robot'),
        "' in ['silver_badger', 'honey_badger'] else '", nav2_unitree_file, "'"
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Load a pre-saved map'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_settings_file,
            'map': LaunchConfiguration('map'),
            'use_localization': 'False',
            # 'use_composition': 'False',
        }.items()
    )


    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        # PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_settings_file,
        }.items()
    )

    # Launch acml and localization if map is provided
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
        launch_arguments={
            # 'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': nav2_settings_file,
        }.items()
    )

    nav2_localization_conditional = GroupAction(
        actions=[
            nav2_localization,
        ],
        condition=LaunchConfigurationNotEquals('map', '')
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_settings_file]
    )
        

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    lifecycle_nodes = ['map_server']
    # lifecycle_nodes = ['map_server', 'amcl']
    use_sim_time = True
    autostart = True

    life_cycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    # Start laser only if map is set
    laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('quadstack_utils'), 'launch', 'depth_to_laser.launch.py')),
        condition=LaunchConfigurationNotEquals('map', '')
    )

    # Change velocity topic name for nav velocity
    velocity_relay = Node(
        package='quadstack_utils',
        executable='velocity_relay',
        name='velocity_relay',
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        use_sim_time_arg,
        map_file_arg,
        # nav2,
        nav2_navigation,
        # nav2_amcl,
        laser,
        map_server,
        life_cycle_manager,
        # nav2_localization_conditional,
        velocity_relay,
    ])
