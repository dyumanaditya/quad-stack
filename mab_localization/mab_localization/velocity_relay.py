import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityRelay(Node):
    def __init__(self):
        super().__init__('velocity_relay')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/hb40/velocity_command', 10)

    def cmd_vel_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
