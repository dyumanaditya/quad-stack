# create a launch file to launch the contact detector node
# subsribe to the /joint_states, /odom to retrieve joint states and odometry
# info of the robot. Then use the contact detector to detect the contact and
# publish the contact state to the /contact_state topic.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
        
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='silver_badger',
            description='Choose the robot to spawn, silver_badger, honey_badger'
        ),
        Node(
            package='quadstack_contact',
            executable='contact_detector_node',
            name='contact_detector_node',
            output='screen',
            parameters=[{'robot': LaunchConfiguration('robot')}],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()