import rclpy
from rclpy.node import Node
from hb40_commons.msg import JointCommand
from sensor_msgs.msg import JointState
import numpy as np
import time


class QuadstackStand(Node):

    def __init__(self):
        super().__init__('quadstack_stand')
        
        self.declare_parameter('robot', 'silver_badger')
        self.robot = self.get_parameter('robot').value
        
        if self.robot == 'silver_badger':
            self.publisher_ = self.create_publisher(JointCommand, '/hb40/joint_commandHighPrio', 10)
            self.target_positions = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5, 0.0]
        elif self.robot == 'honey_badger':
            self.publisher_ = self.create_publisher(JointCommand, '/hb40/joint_commandHighPrio', 10)
            self.target_positions = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5]
        elif self.robot == 'a1':
            self.publisher_ = self.create_publisher(JointCommand, '/a1/joint_commandHighPrio', 10)
            self.target_positions = [-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5]
            # self.target_positions = [0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
        elif self.robot == 'go1':
            self.publisher_ = self.create_publisher(JointCommand, '/go1/joint_commandHighPrio', 10)
            self.target_positions = [-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5]
            # self.target_positions = [0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
        elif self.robot == 'go2':
            self.publisher_ = self.create_publisher(JointCommand, '/go2/joint_commandHighPrio', 10)
            self.target_positions = [-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5]
            # self.target_positions = [0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
        else:
            raise ValueError("Invalid robot name: {}".format(self.robot))
        
        # Subscribe to the joint states
        self.start_joint_state = None
        self.current_joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.first_time = True

    def timer_callback(self):
        if self.first_time and self.start_joint_state is not None:
            self.first_time = False

            # interpolate trajectory
            num_points = 50
            duration = 1.0
            positions_set = self.interpolate_trajectory(self.start_joint_state, self.target_positions, num_points, duration)

            for positions in positions_set:
                msg = JointCommand()
                msg.t_pos = positions
                msg.kd = [5.0] * len(self.target_positions)
                msg.kp = [50.0] * len(self.target_positions)
                self.publisher_.publish(msg)
                time.sleep(duration / num_points)
        else:
            pass
            # msg = JointCommand()
            # msg.t_pos = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5, 0.0]
            # msg.kd = [0.5] * 13
            # msg.kp = [20.0] * 13
            # self.publisher_.publish(msg)

    def interpolate_trajectory(self, start_positions, target_positions, num_points, duration):
        times = np.linspace(0, duration, num_points)
        positions_set = []

        for t in times:
            positions = [
                start + (target - start) * (t / duration)
                for start, target in zip(start_positions, target_positions)
            ]
            positions_set.append(positions)

        return positions_set
    
    def joint_states_callback(self, msg):
        self.start_joint_state = msg.position

def main(args=None):
    rclpy.init(args=args)
    node = QuadstackStand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
