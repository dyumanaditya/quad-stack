import rclpy
from rclpy.node import Node
from hb40_commons.msg import JointCommand
import numpy as np
import time


class MABStand(Node):

    def __init__(self):
        super().__init__('mab_stand')
        self.publisher_ = self.create_publisher(JointCommand, '/hb40/joint_commandHighPrio', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.first_time = True

    def timer_callback(self):
        if self.first_time:
            self.first_time = False
            start_positions = [0.0] * 13
            target_positions = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5, 0.0]

            # interpolate trajectory
            num_points = 50
            duration = 1.0
            positions_set = self.interpolate_trajectory(start_positions, target_positions, num_points, duration)

            for positions in positions_set:
                msg = JointCommand()
                msg.t_pos = positions
                msg.kd = [0.5] * 13
                msg.kp = [30.0] * 13
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

def main(args=None):
    rclpy.init(args=args)
    node = MABStand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
