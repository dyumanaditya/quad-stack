import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class TransformToTopic(Node):
    def __init__(self):
        super().__init__('transform_to_topic')
        
        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers for each required transform
        self.pubs = {
            ('map', 'base'): self.create_publisher(TransformStamped, '/map_base', 10),
            ('odom', 'base'): self.create_publisher(TransformStamped, '/odom_base', 10),
            ('odom_kinematics', 'base'): self.create_publisher(TransformStamped, '/odom_kinematics_base', 10),
            ('map', 'base_link'): self.create_publisher(TransformStamped, '/map_base_link', 10),
            ('odom', 'base_link'): self.create_publisher(TransformStamped, '/odom_base_link', 10),
            ('odom_kinematics', 'base_link'): self.create_publisher(TransformStamped, '/odom_kinematics_base_link', 10),
        }
        
        # Timer to periodically check and publish transforms
        self.timer = self.create_timer(0.005, self.check_transforms)  # 200Hz
    
    def check_transforms(self):
        for (parent_frame, child_frame), publisher in self.pubs.items():
            try:
                # Lookup transform
                transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
                
                # Publish transform
                publisher.publish(transform)
            except Exception as e:
                pass  # Do nothing if the transform is not available


def main(args=None):
    rclpy.init(args=args)
    node = TransformToTopic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
