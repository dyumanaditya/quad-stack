import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations
import math
import time

class FakeIMUPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        
        # Create a publisher for the IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        # Set the publish rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_fake_imu_data)

        # Start time to track the 10-second period
        self.start_time = time.time()

    def publish_fake_imu_data(self):
        # Calculate the elapsed time
        elapsed_time = time.time() - self.start_time

        # Calculate the roll angle in radians based on a sine wave
        # Period is 10 seconds, so omega = 2 * pi / 10
        roll_angle_degrees = 25 * math.sin(2 * math.pi * elapsed_time / 10)
        roll_angle_radians = math.radians(roll_angle_degrees)

        # Convert roll to a quaternion (only roll changes, pitch and yaw are zero)
        quaternion = tf_transformations.quaternion_from_euler(roll_angle_radians, 0.0, 0.0)
        # quaternion = tf_transformations.quaternion_from_euler(0.0, roll_angle_radians, 0.0)

        # Create the IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # The frame ID can be set to whatever is appropriate

        # Set the orientation based on the roll angle
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        # Optionally, you can set the angular velocity and linear acceleration to zero
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        # Publish the IMU message
        self.publisher_.publish(imu_msg)
        self.get_logger().info(f'Published IMU data with roll: {roll_angle_degrees:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = FakeIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
