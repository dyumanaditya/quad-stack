import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MABStandController(Node):

    def __init__(self):
        super().__init__('quadruped_stand_controller')
        
        # Declare each joint angle parameter individually
        self.declare_parameter('sp_j0', 0.0)
        self.declare_parameter('fl_j0', 0.0)
        self.declare_parameter('fr_j0', 0.0)
        self.declare_parameter('fl_j1', 0.0)
        self.declare_parameter('fl_j2', 1.1)
        self.declare_parameter('fr_j1', 0.0)
        self.declare_parameter('rl_j0', 0.0)
        self.declare_parameter('rl_j1', 0.0)
        self.declare_parameter('rl_j2', 0.95)
        self.declare_parameter('fr_j2', -1.1)
        self.declare_parameter('rr_j0', 0.0)
        self.declare_parameter('rr_j1', 0.0)
        self.declare_parameter('rr_j2', -0.95)

        self.standing_pose = {
            'sp_j0': self.get_parameter('sp_j0').value,
            'fr_j0': self.get_parameter('fr_j0').value,
            'fr_j1': self.get_parameter('fr_j1').value,
            'fr_j2': self.get_parameter('fr_j2').value,
            'fl_j0': self.get_parameter('fl_j0').value,
            'fl_j1': self.get_parameter('fl_j1').value,
            'fl_j2': self.get_parameter('fl_j2').value,
            'rl_j0': self.get_parameter('rl_j0').value,
            'rl_j1': self.get_parameter('rl_j1').value,
            'rl_j2': self.get_parameter('rl_j2').value,
            'rr_j0': self.get_parameter('rr_j0').value,
            'rr_j1': self.get_parameter('rr_j1').value,
            'rr_j2': self.get_parameter('rr_j2').value,
        }

        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/joint_trajectory_controller/commands', 10)

        self.current_joint_states = None
        self.error_sum = {name: 0.0 for name in self.standing_pose.keys()}
        self.previous_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def calculate_efforts(self):
        if self.current_joint_states is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.previous_time = current_time

        # PI-controller to calculate efforts
        efforts = []
        k_p = 2.0  # Proportional gain
        k_i = 1.0   # Integral gain

        for name in self.current_joint_states.name:
            if name in self.standing_pose:
                target_angle = self.standing_pose[name]
                current_state_idx = self.current_joint_states.name.index(name)
                current_angle = self.current_joint_states.position[current_state_idx]
                error = target_angle - current_angle
                
                # Proportional term
                p_term = k_p * error
                
                # Integral term
                self.error_sum[name] += error * dt
                i_term = k_i * self.error_sum[name]
                
                # Total effort
                effort = p_term + i_term
                effort = np.clip(effort, -16.0, 16.0)
                if name == 'sp_j0':
                    effort *= 2.0
                efforts.append(effort)
            else:
                efforts.append(0.0)
        
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts
        self.effort_publisher.publish(effort_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MABStandController()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.calculate_efforts()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
