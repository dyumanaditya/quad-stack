import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class LaserFrameProjector(Node):
    def __init__(self):
        super().__init__('laser_frame_projector')

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically call the function
        self.timer = self.create_timer(0.01, self.broadcast_transform)
        

    def broadcast_transform(self):
        try:
            # Get the current transform from "base_link" to "laser_frame"
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform('odom', 'laser_frame', rclpy.time.Time())

            # Modify the transform by projecting onto the XY plane (setting z to 0)
            projected_transform = TransformStamped()
            projected_transform.header.stamp = now.to_msg()
            projected_transform.header.frame_id = trans.header.frame_id
            projected_transform.child_frame_id = "laser_frame_projected"

            projected_transform.transform.translation.x = trans.transform.translation.x
            projected_transform.transform.translation.y = trans.transform.translation.y
            projected_transform.transform.translation.z = 0.0  # Project to the ground

            # Convert the current quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])

            # Set roll and pitch to zero (only keep yaw)
            new_quat = quaternion_from_euler(0.0, 0.0, yaw)

            # Set the new orientation
            projected_transform.transform.rotation.x = new_quat[0]
            projected_transform.transform.rotation.y = new_quat[1]
            projected_transform.transform.rotation.z = new_quat[2]
            projected_transform.transform.rotation.w = new_quat[3]

            # Broadcast the new transform
            self.br.sendTransform(projected_transform)

        except Exception as e:
            self.get_logger().warn(f'Could not transform laser_frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
