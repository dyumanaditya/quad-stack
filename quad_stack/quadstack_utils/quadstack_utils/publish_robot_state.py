import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from hb40_commons.msg import BridgeData
from time import time
from rclpy.clock import Clock, ClockType




class PublishRobotState(Node):
    def __init__(self):
        super().__init__('publish_robot_state')


        self.pose_sub = self.create_subscription(
            Imu,
            '/imu/out',
            self.orientation_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.declare_parameter("robot", "silver_badger")
        self.robot = self.get_parameter("robot").get_parameter_value().string_value
        
        # Joints in the correct order (as expected by hb40)
        if self.robot == "silver_badger":
            self.joints = ['fr_j0', 'fr_j1', 'fr_j2', 'fl_j0', 'fl_j1', 'fl_j2', 'rl_j0', 'rl_j1', 'rl_j2', 'rr_j0', 'rr_j1', 'rr_j2', 'sp_j0']
        else:
            self.joints = ['fr_j0', 'fr_j1', 'fr_j2', 'fl_j0', 'fl_j1', 'fl_j2', 'rl_j0', 'rl_j1', 'rl_j2', 'rr_j0', 'rr_j1', 'rr_j2', 'fixed_spine']
            

        self.publisher = self.create_publisher(BridgeData, '/hb40/bridge_data', 10)
        timer_period = 0.01  # seconds (100 Hz)
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(timer_period, self.publish_combined_message, clock=self.system_clock)

        self.current_orientation = None
        self.current_angular_vel = None
        self.current_joints_pos = None
        self.current_joints_vel = None
        self.current_joints_effort = None

    def orientation_callback(self, msg):
        # Get the orientation and angular velocity
        self.current_orientation = msg.orientation
        self.current_angular_vel = msg.angular_velocity
        # self.publish_combined_message()

    def joint_callback(self, msg):
        # Get the joint states (in the correct order)
        self.current_joints_pos = [0.0] * len(self.joints)
        self.current_joints_vel = [0.0] * len(self.joints)
        self.current_joints_effort = [0.0] * len(self.joints)
        for i, joint_name in enumerate(msg.name):
            joint_idx = self.joints.index(joint_name)
            self.current_joints_pos[joint_idx] = msg.position[i]
            self.current_joints_vel[joint_idx] = msg.velocity[i]
            self.current_joints_effort[joint_idx] = msg.effort[i]
            
        if self.robot == "honey_badger":
            # Honey badger has a fixed spine joint, the controller expects 13 inputs
            self.current_joints_pos[-1] = 0.0
            self.current_joints_vel[-1] = 0.0
            self.current_joints_effort[-1] = 0.0

        # self.publish_combined_message()

    def publish_combined_message(self):
        if self.current_orientation is not None and self.current_angular_vel is not None and self.current_joints_pos is not None and self.current_joints_vel is not None and self.current_joints_effort is not None:
            robot_state = BridgeData()
            robot_state.header.stamp = self.get_clock().now().to_msg()
            robot_state.orientation.x = self.current_orientation.x
            robot_state.orientation.y = self.current_orientation.y
            robot_state.orientation.z = self.current_orientation.z
            robot_state.orientation.w = self.current_orientation.w
            
            # For now set linear and angular velocity to 0 (use odom later)
            robot_state.angular_velocity.x = self.current_angular_vel.x
            robot_state.angular_velocity.y = self.current_angular_vel.y
            robot_state.angular_velocity.z = self.current_angular_vel.z

            robot_state.joint_name = self.joints
            robot_state.joint_position = self.current_joints_pos
            robot_state.joint_velocity = self.current_joints_vel
            robot_state.joint_effort = self.current_joints_effort
            self.publisher.publish(robot_state)

def main(args=None):
    rclpy.init(args=args)
    node = PublishRobotState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
