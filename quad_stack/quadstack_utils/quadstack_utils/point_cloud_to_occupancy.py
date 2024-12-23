import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sensor_msgs_py import point_cloud2



class PointCloudToOccupancyGrid(Node):

    def __init__(self):
        super().__init__('pointcloud_to_occupancy_grid')
        
        # Parameters
        self.declare_parameter('pointcloud_topic', '/cloud_obstacles')
        self.declare_parameter('occupancy_grid_topic', '/map_flattened')
        self.declare_parameter('height_threshold', 1.0)
        self.declare_parameter('grid_resolution', 0.05)  # Resolution of the occupancy grid (meters/cell)
        self.declare_parameter('intensity_threshold', 3)   # Only accept points with intensity greater than this value

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').get_parameter_value().string_value
        self.height_threshold = self.get_parameter('height_threshold').get_parameter_value().double_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.intensity_threshold = self.get_parameter('intensity_threshold').get_parameter_value().integer_value
        
        # Subscriber and Publisher
        self.subscriber = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.publisher = self.create_publisher(OccupancyGrid, self.occupancy_grid_topic, 10)

    def pointcloud_callback(self, msg: PointCloud2):
        cloud = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        filtered_cloud = cloud[cloud[:, 2] < self.height_threshold]

        # Now make the occupancy grid
        min_x = np.min(filtered_cloud[:, 0])
        min_y = np.min(filtered_cloud[:, 1])
        max_x = np.max(filtered_cloud[:, 0])
        max_y = np.max(filtered_cloud[:, 1])

        grid_width = int((max_x - min_x) / self.grid_resolution) + 1
        grid_height = int((max_y - min_y) / self.grid_resolution) + 1

        # Initialize the occupancy grid (set all cells to "unknown", e.g., -1)
        occupancy_grid = np.full((grid_height, grid_width), -1, dtype=np.int8)

        # Map points to grid indices
        grid_indices_x = ((filtered_cloud[:, 0] - min_x) / self.grid_resolution).astype(np.int32)
        grid_indices_y = ((filtered_cloud[:, 1] - min_y) / self.grid_resolution).astype(np.int32)

        # Mark cells as occupied
        for i in range(len(filtered_cloud)):
            if 0 <= grid_indices_x[i] < grid_width and 0 <= grid_indices_y[i] < grid_height:
                occupancy_grid[grid_indices_y[i], grid_indices_x[i]] += 1 # Mark as occupied

        # Filter out anything that is less than intensity threshold
        occupancy_grid[occupancy_grid < self.intensity_threshold] = -1

        # Publish the occupancy grid
        self.publish_occupancy_grid(occupancy_grid, msg.header, min_x, min_y, grid_width, grid_height)

    def publish_occupancy_grid(self, grid, header, min_x, min_y, grid_width, grid_height):
        # Convert numpy grid to OccupancyGrid message
        occupancy_msg = OccupancyGrid()
        occupancy_msg.header = header
        occupancy_msg.info.resolution = self.grid_resolution
        occupancy_msg.info.width = grid_width
        occupancy_msg.info.height = grid_height
        occupancy_msg.info.origin.position.x = float(min_x)
        occupancy_msg.info.origin.position.y = float(min_y)
        occupancy_msg.info.origin.position.z = 0.0
        occupancy_msg.info.origin.orientation.w = 1.0
        
        # Flatten the grid and convert it to a list
        occupancy_msg.data = grid.flatten().tolist()
        
        # Publish the occupancy grid
        self.publisher.publish(occupancy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
