import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityCommandPublisher(Node):
    def __init__(self):
        super().__init__('velocity_command_publisher')

        # Declare and initialize parameters
        self.declare_parameter('mode', 'straight_line')
        self.linear_velocity = 0.155  # meters/second
        self.angular_velocity = 0.153  # radians/second

        # Retrieve mode parameter
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to publish at a fixed rate
        self.timer = self.create_timer(0.2, self.publish_velocity_command)

        self.get_logger().info(f"Velocity command publisher initialized in '{self.mode}' mode.")

    def publish_velocity_command(self):
        twist = Twist()

        if self.mode == 'straight_line':
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

        elif self.mode == 'circle':
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity

        elif self.mode == 'eight':
            time_now = self.get_clock().now().to_msg().sec
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity * 2 * (-1 if (time_now // 30) % 2 == 0 else 1)
        else:
            self.get_logger().warn(f"Unknown mode '{self.mode}'. Defaulting to 'straight_line'.")
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityCommandPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        velocity_publisher.get_logger().info("Node interrupted. Shutting down.")
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
