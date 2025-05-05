import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition



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
    
    use_kinematics_odom_arg = DeclareLaunchArgument(
        'use_kinematics_odom',
        default_value='true',
        description='Use kinematics odometry'
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
    
    # Whether to use kinematics odometry or not
    use_kinematics_odom = LaunchConfiguration('use_kinematics_odom')
    kinematics_frame = PythonExpression([
        "'odom_kinematics' if '", use_kinematics_odom,
        "' == 'true' else ''"
    ])
    
    vo_params = PythonExpression([
        "'vo_params_real.yaml' if '", LaunchConfiguration('real_robot'), "' == 'true' else 'vo_params_sim.yaml'"
    ])
    vo_settings_file = [os.path.join(pkg_share, 'resource', ''), vo_params]

    
    rtabmap = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=
        [
            {
            'frame_id': frame_id,
            'guess_frame_id': kinematics_frame,  
            },
            vo_settings_file
        ],
        remappings=[
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_rect_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
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
    
    # launch the contact detector node
    use_contact_detector = PythonExpression([
        "'true' if '", LaunchConfiguration('robot'),
        "' in ['silver_badger', 'honey_badger'] and '",
        LaunchConfiguration('real_robot'),
        "' == 'true' else 'false'"
    ])
    contact_detector = Node(
        package='quadstack_contact',
        executable='contact_detector_node',
        name='contact_detector_node',
        output='screen',
        parameters=[{'robot': LaunchConfiguration('robot')}],
        condition=IfCondition(use_contact_detector)
    )

    kinematics_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kinematics_odom_pkg_share, 'launch', 'kinematics_odometry.launch.py')),
        launch_arguments={'urdf': urdf_path_expr, 'robot': LaunchConfiguration('robot'), 'real_robot': LaunchConfiguration('real_robot')}.items(),
        condition=IfCondition(use_kinematics_odom)
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

    rtabmap_vis = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        output='screen',
    )
    
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base',
            'approx_sync': True,
            'odom_frame_id': 'odom',
        }],
        remappings=[
            ('/rgb/image', '/d435i_camera/color/image_raw'),
            ('/depth/image', '/d435i_camera/depth/image_rect_raw'),
            ('/rgb/camera_info', '/d435i_camera/color/camera_info'),
            ('/imu', '/imu/out'),
        ]
    )


    return LaunchDescription([
        robot_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_kinematics_odom_arg,
        rtabmap,
        # odom_gt,
        # odom_2d,
        contact_detector,
        kinematics_odometry,
        # imu_covariance,
        # ekf_filter,
        # rtabmap_vis,
        # rtabmap_slam
    ])
