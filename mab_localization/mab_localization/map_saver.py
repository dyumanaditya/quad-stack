import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning

    def map_callback(self, msg):
        # Convert occupancy grid to numpy array
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Convert to 8-bit grayscale image
        image = np.zeros((height, width), dtype=np.uint8)
        image[data == -1] = 127  # Unknown
        image[data == 0] = 255  # Free
        image[data == 100] = 0  # Occupied

        # Save image
        cv2.imwrite('/home/aditya/map.png', image)
        # self.get_logger().info('Map image saved as map_image.png')

        # Optionally shut down after saving
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
