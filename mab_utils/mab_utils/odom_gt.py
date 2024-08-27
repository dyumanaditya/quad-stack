import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomGTNode(Node):
    def __init__(self):
        super().__init__('odom_gt_node')
        self.subscription = self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):        
        # Publish the corresponding transform from odom to base_frame
        self.publish_transform(msg)

    def publish_transform(self, odom_msg):
        transform_base_footprint = TransformStamped()
        
        transform_base_footprint.header.stamp = odom_msg.header.stamp
        transform_base_footprint.header.frame_id = "odom_gt"  # 'odom' frame
        transform_base_footprint.child_frame_id = "base_link"    # 'base_frame' or 'base_link'

        # Set the translation
        transform_base_footprint.transform.translation.x = odom_msg.pose.pose.position.x
        transform_base_footprint.transform.translation.y = odom_msg.pose.pose.position.y
        transform_base_footprint.transform.translation.z = odom_msg.pose.pose.position.z
        
        # Set the rotation
        transform_base_footprint.transform.rotation = odom_msg.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform_base_footprint)

def main(args=None):
    rclpy.init(args=args)
    node = OdomGTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
