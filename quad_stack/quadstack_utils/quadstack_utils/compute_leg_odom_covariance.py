#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

# For time-synchronized callbacks, we'll use message_filters.
from message_filters import ApproximateTimeSynchronizer, Subscriber

class CovarianceEstimator(Node):
    def __init__(self):
        super().__init__('covariance_estimator')

        # Create message_filters subscribers for both topics
        self.base_vel_sub = Subscriber(self, TwistWithCovarianceStamped, '/base_lin_vel')
        self.odom_sub = Subscriber(self, Odometry, '/odom_gt')
        
        # Use ApproximateTimeSynchronizer to sync the messages (tweak queue size and slop as needed)
        self.ts = ApproximateTimeSynchronizer(
            [self.base_vel_sub, self.odom_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        # Variables for online covariance estimation (Welford's algorithm)
        self.n = 0
        self.mean = np.zeros(3)  # assuming 3D velocity (x, y, z)
        self.M2 = np.zeros((3, 3))

    def callback(self, base_lin_vel_msg: TwistWithCovarianceStamped, odom_gt_msg: Odometry):
        # Extract linear velocity from the /base_lin_vel topic
        v_est = np.array([
            base_lin_vel_msg.twist.twist.linear.x,
            base_lin_vel_msg.twist.twist.linear.y,
            base_lin_vel_msg.twist.twist.linear.z
        ])

        # Extract the ground truth linear velocity from the /odom_gt topic
        v_gt = np.array([
            odom_gt_msg.twist.twist.linear.x,
            odom_gt_msg.twist.twist.linear.y,
            odom_gt_msg.twist.twist.linear.z
        ])

        # Compute the error between the computed velocity and ground truth.
        error = v_est - v_gt

        # Incremental covariance update using Welford's method
        self.n += 1
        if self.n == 1:
            self.mean = error
        else:
            delta = error - self.mean
            self.mean += delta / self.n
            self.M2 += np.outer(delta, error - self.mean)

        # Once we have at least two samples, compute and print the sample covariance matrix.
        if self.n > 1:
            cov = self.M2 / (self.n - 1)
            self.get_logger().info('Current covariance matrix:\n{}'.format(cov))
        else:
            self.get_logger().info('Collecting data... n = {}'.format(self.n))

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
