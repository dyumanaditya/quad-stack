from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    # Mode param
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='straight_line',
        description='Mode of operation for velocity publisher: straight_line, circle, or eight'
    )
    
    publish_velocity_path = Node(
        package='quadstack_utils',
        executable='publish_velocity_path',
        name='publish_velocity_path_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode')
        }]
    )

    return LaunchDescription([
        mode_arg,
        publish_velocity_path,
    ])
