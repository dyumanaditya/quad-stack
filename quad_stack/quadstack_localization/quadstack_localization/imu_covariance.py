import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuWithCovarianceNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/out',  # Replace with your IMU topic name
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, '/imu', 10)

    def imu_callback(self, msg):
        # Set the covariance values
        msg.orientation_covariance = [
            10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 10.0
        ]
        msg.angular_velocity_covariance = [
            20.0, 0.0, 0.0,
            0.0, 20.0, 0.0,
            0.0, 0.0, 20.0
        ]
        msg.linear_acceleration_covariance = [
            30.0, 0.0, 0.0,
            0.0, 30.0, 0.0,
            0.0, 0.0, 30.0
        ]

        # Publish the message with covariance
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuWithCovarianceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
