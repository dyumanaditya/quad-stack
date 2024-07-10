import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np



class MABStand(Node):

    def __init__(self):
        super().__init__('mab_stand')
        
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory_once)
        self.already_published = False

        # Define target joint positions for standing
        self.target_positions = {
            'sp_j0': 0.0,
            'fr_j0': -0.1,
            'fr_j1': 0.8,
            'fr_j2': -1.5,
            'fl_j0': 0.1,
            'fl_j1': -0.8,
            'fl_j2': 1.5,
            'rl_j0': -0.1,
            'rl_j1': -1.0,
            'rl_j2': 1.5,
            'rr_j0': 0.1,
            'rr_j1': 1.0,
            'rr_j2': -1.5,
        }

        self.num_points = 50  # Number of points for interpolation
        self.duration = 2.0   # Duration of the trajectory in seconds

        self.joint_names = list(self.target_positions.keys())
        self.start_positions = [0.0] * len(self.joint_names)

    def interpolate_trajectory(self, start_positions, target_positions, num_points, duration):
        times = np.linspace(0, duration, num_points)
        trajectory_points = []

        for t in times:
            positions = [
                start + (target - start) * (t / duration)
                for start, target in zip(start_positions, target_positions)
            ]

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = rclpy.time.Duration(seconds=t).to_msg()
            trajectory_points.append(point)

        return trajectory_points

    def publish_trajectory_once(self):
        if self.already_published:
            return
        
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        target_positions = [self.target_positions[name] for name in self.joint_names]
        joint_trajectory_msg.points = self.interpolate_trajectory(
            self.start_positions, target_positions, self.num_points, self.duration
        )

        self.joint_trajectory_publisher.publish(joint_trajectory_msg)
        self.get_logger().info('Published joint trajectory to stand')
        
        self.already_published = True
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MABStand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
