import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Odom2DNode(Node):
    def __init__(self):
        super().__init__('odom_2d_node')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/odom_2d', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.odom_2d = None

        # self.create_timer(0.01, self.publish_transform)

    def odom_callback(self, msg):
        # Extract the 3D pose
        pose = msg.pose.pose
        
        # Convert the quaternion to Euler angles
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        # Create a new Odometry message for 2D
        odom_2d = Odometry()
        odom_2d.header = msg.header
        odom_2d.child_frame_id = msg.child_frame_id
        
        # Only set the x, y and yaw components
        odom_2d.pose.pose.position.x = pose.position.x
        odom_2d.pose.pose.position.y = pose.position.y
        odom_2d.pose.pose.position.z = 0.0  # Set z to 0 for 2D plane

        # Convert yaw back to a quaternion, with roll and pitch set to 0
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom_2d.pose.pose.orientation.x = quat[0]
        odom_2d.pose.pose.orientation.y = quat[1]
        odom_2d.pose.pose.orientation.z = quat[2]
        odom_2d.pose.pose.orientation.w = quat[3]
        
        # Set the twist, but only consider the linear x, y and angular z components
        odom_2d.twist.twist.linear.x = msg.twist.twist.linear.x
        odom_2d.twist.twist.linear.y = msg.twist.twist.linear.y
        odom_2d.twist.twist.linear.z = 0.0
        odom_2d.twist.twist.angular.x = 0.0
        odom_2d.twist.twist.angular.y = 0.0
        odom_2d.twist.twist.angular.z = msg.twist.twist.angular.z
        
        # Publish the 2D odometry
        self.publisher.publish(odom_2d)
        
        # Publish the corresponding transform from odom to base_frame
        self.publish_transform(odom_2d)

    def publish_transform(self, odom_2d):
        transform_base_footprint = TransformStamped()
        
        transform_base_footprint.header.stamp = odom_2d.header.stamp
        transform_base_footprint.header.frame_id = odom_2d.header.frame_id  # 'odom' frame
        transform_base_footprint.child_frame_id = "base_footprint"    # 'base_frame' or 'base_link'

        # Set the translation
        transform_base_footprint.transform.translation.x = odom_2d.pose.pose.position.x
        transform_base_footprint.transform.translation.y = odom_2d.pose.pose.position.y
        transform_base_footprint.transform.translation.z = 0.0  # Stay on the 2D plane
        
        # Set the rotation
        transform_base_footprint.transform.rotation = odom_2d.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform_base_footprint)

def main(args=None):
    rclpy.init(args=args)
    node = Odom2DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
