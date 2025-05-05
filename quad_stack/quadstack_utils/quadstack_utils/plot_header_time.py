#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

# Import the message type. For this example, we assume both topics publish messages
# that contain a header similar to nav_msgs/Odometry.
from nav_msgs.msg import Odometry
from hb40_commons.msg import RobotState, BridgeData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class PlotHeaderTimeNode(Node):
    def __init__(self):
        super().__init__('plot_header_time_node')
        # Lists to store the header times from each topic.
        self.odom_times = []
        self.robot_state_times = []
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Matches publisher
            durability=QoSDurabilityPolicy.VOLATILE,  # Matches publisher
            depth=10  # Depth is unknown, so we set a reasonable value
        )

        # Subscribe to the '/optitrack/odom' topic.
        self.create_subscription(
            Odometry,
            '/d435i_camera/depth/image_rect_raw',
            self.odom_callback,
            10)

        # Subscribe to the '/hb40/robot_state' topic.
        self.create_subscription(
            BridgeData,
            '/hb40/bridge_data',
            self.robot_state_callback,
            qos_profile)

        self.get_logger().info("PlotHeaderTimeNode started and subscribed to topics.")

    def odom_callback(self, msg):
        # Extract the time from the header and convert to seconds.
        header_stamp = msg.header.stamp
        time_sec = header_stamp.sec + header_stamp.nanosec * 1e-9
        self.odom_times.append(time_sec)

    def robot_state_callback(self, msg):
        header_stamp = msg.header.stamp
        time_sec = header_stamp.sec + header_stamp.nanosec * 1e-9
        self.robot_state_times.append(time_sec)

    def plot_times(self):
        plt.figure()
        # Plot times from /optitrack/odom in red
        plt.scatter(self.odom_times, [1]*len(self.odom_times), color='red', label='/d435i_camera/depth/image_rect_raw')
        # Plot times from /hb40/robot_state in blue
        plt.scatter(self.robot_state_times, [1]*len(self.robot_state_times), color='blue', label='/hb40/bridge_data')

        plt.xlabel("Header Time (seconds)")
        plt.ylabel("Value")
        plt.title("Header Times from Topics")
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlotHeaderTimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down node...")
    finally:
        # When shutting down, plot the collected header times.
        node.get_logger().info("Plotting collected header times before exit.")
        node.plot_times()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
