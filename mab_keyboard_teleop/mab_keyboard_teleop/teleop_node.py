import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time

# Key mappings
move_bindings = {
    'w': (0.15, 0),
    's': (-0.15, 0),
    'a': (0, 0.15),
    'd': (0, -0.15),
    'f': (0, 0),
    'q': (0, 0)
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/hb40/velocity_command', 10)
        self.get_logger().info('Teleop Node has been started.')
        self.settings = termios.tcgetattr(sys.stdin)
        self.zero_twist = Twist()
        self.zero_twist.linear.x = 0.0
        self.zero_twist.angular.z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0

        # Create a timer to publish zero twist
        self.timer = self.create_timer(0.2, self.publish_zero_twist)
        self.key_pressed = False

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_zero_twist(self):
        if not self.key_pressed:
            self.publisher_.publish(self.zero_twist)
        self.key_pressed = False

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key in move_bindings.keys():
                    x, y = move_bindings[key]
                    twist = Twist()
                    self.current_x += x
                    self.current_y += y
                    if key == 'f':
                        self.current_x = 0.0
                        self.current_y = 0.0
                    twist.linear.x = float(self.current_x)
                    twist.angular.z = float(self.current_y)
                    self.publisher_.publish(twist)
                    self.key_pressed = True
                    if key == 'q':
                        break
                elif key:
                    self.get_logger().info('Unknown key pressed')
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            self.publisher_.publish(self.zero_twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
