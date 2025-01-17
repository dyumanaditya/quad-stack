import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_name = "quadstack_localization"
    kinematics_odom_pkg_name = "legged_kinematics_odometry"
    pkg_share = get_package_share_directory(pkg_name)
    kinematics_odom_pkg_share = get_package_share_directory(kinematics_odom_pkg_name)
    ekf_settings_file = os.path.join(pkg_share, 'resource', 'ekf.yaml')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='silver_badger',
        description='Choose the robot to spawn, silver_badger, honey_badger, a1, go1 or go2'
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
    
    # Use LaunchConfiguration to fetch the values of the launch arguments
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Combine the pose values into the initial_pose string
    # With correct spacing and formatting
    initial_pose = [x_pose, ' ', y_pose, ' ', z_pose, ' ', '0 ', '0 ', '0']
    frame_id = PythonExpression([
        "'base_link' if '", LaunchConfiguration('robot'), "' in ['silver_badger', 'honey_badger'] else 'base'"
    ])
    
    rtabmap = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'approx_sync': True,
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'initial_pose': '0 0 0 0 0 0',
            # 'initial_pose': '-2.0 3.5 0 0 0 0',
            # 'initial_pose': initial_pose,
            'publish_null_when_lost': False,
            # 'odom_frame_id': 'odom_turtlebot3',
            # 'queue_size': 1000,
            # 'wait_for_transform': 1.0,
            'approx_sync_max_interval': 0.05,
            # 'sensor_data_compression_format': 'jpeg',
            # 'topic_queue_size': 1000,
            'Odom/Strategy': '0', # Frame to map or Frame to frame
            'Vis/CorType': '0', # Features Matching or Optical flow
            'OdomF2M/MaxSize': '1200', # Max features
            'Vis/MaxFeatures': '700', # Max features extracted from image
            'wait_imu_to_init': True,
            'guess_frame_id': 'odom_kinematics',
        }],
        remappings=[
            # ('/rgb/image', '/d435i_camera/color/image_raw/rotated'),
            # ('/depth/image', '/d435i_camera/depth/image_raw/rotated'),
            # ('/rgb/camera_info', '/d435i_camera/color/camera_info/rotated')
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_rect_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            # ('/rgb/image', '/d435i_camera/color/image_raw/stabilized'),
            # ('/depth/image', '/d435i_camera/depth/image_rect_raw/stabilized'),
            # ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            # ('/depth/image', '/d435i_camera/depth/image_raw'),
            # ('/rgb/image', '/waffle_cam/color/image_raw'),
            # ('/depth/image', '/waffle_cam/depth/image_raw'),
            # ('/rgb/camera_info', '/waffle_cam/color/camera_info')
            ('/imu', '/imu/out'),
        ]
    )
    
    # Determine the URDF path based on the robot
    sb_description_pkg_share = get_package_share_directory('silver_badger_description')
    hb_description_pkg_share = get_package_share_directory('honey_badger_description')
    a1_description_pkg_share = get_package_share_directory('a1_description')
    go1_description_pkg_share = get_package_share_directory('go1_description')
    go2_description_pkg_share = get_package_share_directory('go2_description')
    
    urdf_paths = {
        'silver_badger': os.path.join(sb_description_pkg_share, 'urdf', 'silver_badger.urdf'),
        'honey_badger': os.path.join(hb_description_pkg_share, 'urdf', 'honey_badger.urdf'),
        'a1': os.path.join(a1_description_pkg_share, 'urdf', 'a1.urdf'),
        'go1': os.path.join(go1_description_pkg_share, 'urdf', 'go1.urdf'),
        'go2': os.path.join(go2_description_pkg_share, 'urdf', 'go2.urdf'),
    }
    
    # Generate a URDF path selection expression
    urdf_path_expr = PythonExpression([
        "dict(",
        str(urdf_paths),
        ").get('",
        LaunchConfiguration('robot'),
        "', '')"
    ])

    kinematics_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kinematics_odom_pkg_share, 'launch', 'kinematics_odometry.launch.py')),
        launch_arguments={'urdf': urdf_path_expr, 'robot': LaunchConfiguration('robot')}.items()
    )

    odom_gt = Node(
        package='quadstack_utils',
        executable='odom_gt',
        name='odom_gt',
        output='screen',
    )

    odom_2d = Node(
        package='mab_localization',
        executable='odom_2d',
        name='odom_2d',
        output='screen',
    )

    imu_covariance = Node(
        package='mab_localization',
        executable='imu_covariance_node',
        name='imu_covariance_node',
        output='screen',
    )

    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_settings_file],
    )

    # rtabmap_vis = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmap_viz',
    #     name='rtabmapviz',
    #     output='screen',
    # )


    return LaunchDescription([
        robot_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        rtabmap,
        # odom_gt,
        # odom_2d,
        kinematics_odometry,
        # imu_covariance,
        # ekf_filter,
        # rtabmap_vis,
    ])
