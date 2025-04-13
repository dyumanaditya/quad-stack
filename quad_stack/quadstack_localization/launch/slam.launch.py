import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression



def generate_launch_description():
    pkg_name = "quadstack_localization"
    utils_pkg_name = "quadstack_utils"
    pkg_share = get_package_share_directory(pkg_name)
    utils_pkg_share = get_package_share_directory(utils_pkg_name)
    # laser_settings_file = os.path.join(pkg_share, 'resource', 'laser.yaml')
    
    slam_params = PythonExpression([
        "'slam_toolbox_params_real.yaml' if '", LaunchConfiguration('real_robot'), "' == 'true' else 'slam_toolbox_params_sim.yaml'"
    ])
    slam_settings_file = [os.path.join(pkg_share, 'resource', ''), slam_params]

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
    )
    
    # Set base_frame dynamically based on the robot argument
    base_frame = PythonExpression([
        "'base_link' if '", LaunchConfiguration('robot'), "' in ['silver_badger', 'honey_badger'] else 'base'"
    ])

    depth_to_laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(utils_pkg_share, 'launch', 'depth_to_laser.launch.py')),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'rosbag': LaunchConfiguration('rosbag'),
            'real_robot': LaunchConfiguration('real_robot'),
            'use_laser_stabilization': LaunchConfiguration('use_laser_stabilization')
        }.items()
    )

    # Whether to add kinematics induced velocity constraints to the map
    use_vel_map_constraints_arg = LaunchConfiguration('use_vel_map_constraints')
    use_vel_map_constraints = PythonExpression([
        "True if '", use_vel_map_constraints_arg,
        "' == 'true' else False"
    ])
    
    # SLAM Node
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_settings_file,  # Pass the settings file as a string
            {
            'base_frame': base_frame,
            'use_velocity_constraints': use_vel_map_constraints,
            }
        ]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'approx_sync': True,
            'odom_frame_id': 'odom',
            # 'queue_size': 1000,
            # 'wait_for_transform': 1.0,
            # 'approx_sync_max_interval': 0.5,
            # 'Odom/Strategy': '1', # Frame to map or Frame to frame
            # 'Vis/CorType': '1', # Features Matching or Optical flow
            # 'OdomF2M/MaxSize': '1000', # Max features
            # 'Vis/MaxFeatures': '600', # Max features extracted from image
            # 'Grid/3D': 'False',
            # 'subscribe_depth': False,
            # 'subscribe_scan': True,
            # 'Grid/Sensor': '1',
        }],
        remappings=[
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            ('/imu', '/imu/out'),
            ('/scan', '/scan')
            # ('/rgb/image', '/waffle_cam/color/image_raw'),
            # ('/depth/image', '/waffle_cam/depth/image_raw'),
            # ('/rgb/camera_info', '/waffle_cam/color/camera_info')
            # ('/imu', '/imu'),
        ],
        arguments=['--delete_db_on_start']

    )

    point_cloud_to_occupancy_grid = Node(
        package='mab_utils',
        executable='point_cloud_to_occupancy',
        name='point_cloud_to_occupancy',
        output='screen',
        parameters=[{
            'pointcloud_topic': '/cloud_obstacles',
            'occupancy_grid_topic': '/map_flattened',
            'height_threshold': 0.1,
            'grid_resolution': 0.1,
            'intensity_threshold': 7
        }]
    )

    return LaunchDescription([
        robot_arg,
        depth_to_laser,
        slam,
        # rtabmap_slam,
        # point_cloud_to_occupancy_grid,
    ])
