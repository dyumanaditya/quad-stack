import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageRotateNode(Node):
    def __init__(self):
        super().__init__('image_rotate_node')
        
        # Depth image and camera info subscribers and publishers
        self.subscription_depth_image = self.create_subscription(
            Image,
            '/input_depth_image',
            self.listener_callback_depth_image,
            10)
        self.subscription_depth_camera_info = self.create_subscription(
            CameraInfo,
            '/input_depth_camera_info',
            self.listener_callback_depth_camera_info,
            10)
        self.publisher_depth_image = self.create_publisher(Image, '/rotated_depth_image', 10)
        self.publisher_depth_camera_info = self.create_publisher(CameraInfo, '/rotated_depth_camera_info', 10)
        
        # Color image and camera info subscribers and publishers
        self.subscription_color_image = self.create_subscription(
            Image,
            '/input_rgb_image',
            self.listener_callback_color_image,
            10)
        self.subscription_color_camera_info = self.create_subscription(
            CameraInfo,
            '/input_rgb_camera_info',
            self.listener_callback_color_camera_info,
            10)
        self.publisher_color_image = self.create_publisher(Image, '/rotated_rgb_image', 10)
        self.publisher_color_camera_info = self.create_publisher(CameraInfo, '/rotated_rgb_camera_info', 10)
        
        self.br = CvBridge()
        
        # Initialize camera info messages
        self.depth_camera_info_msg = None
        self.color_camera_info_msg = None

    def listener_callback_depth_image(self, msg):
        self.process_and_publish_image(msg, self.publisher_depth_image)

    def listener_callback_color_image(self, msg):
        self.process_and_publish_image(msg, self.publisher_color_image, encoding='rgb8')

    def listener_callback_depth_camera_info(self, msg):
        self.depth_camera_info_msg = msg
        self.publish_rotated_camera_info(msg, self.publisher_depth_camera_info)

    def listener_callback_color_camera_info(self, msg):
        self.color_camera_info_msg = msg
        self.publish_rotated_camera_info(msg, self.publisher_color_camera_info)

    def process_and_publish_image(self, msg, publisher, encoding='passthrough'):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding=encoding)
            # Rotate image by -90 degrees (counterclockwise)
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # Convert OpenCV image back to ROS Image message
            rotated_msg = self.br.cv2_to_imgmsg(rotated_image, encoding=encoding)
            rotated_msg.header = msg.header
            # Publish the rotated image
            publisher.publish(rotated_msg)
        
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def publish_rotated_camera_info(self, camera_info_msg, publisher):
        camera_info = CameraInfo()
        camera_info.header = camera_info_msg.header
        camera_info.height = camera_info_msg.width  # Swap height and width
        camera_info.width = camera_info_msg.height  # Swap height and width

        # Rotate the camera matrix (intrinsic parameters)
        K = np.array(camera_info_msg.k).reshape((3, 3))
        K_rotated = np.zeros_like(K)
        K_rotated[0, 0] = K[1, 1]  # fx_rot = fy
        K_rotated[1, 1] = K[0, 0]  # fy_rot = fx
        K_rotated[0, 2] = K[1, 2]  # cx_rot = cy
        K_rotated[1, 2] = K[0, 2]  # cy_rot = cx
        K_rotated[2, 2] = 1.0
        camera_info.k = K_rotated.flatten().tolist()

        # Rotate the projection matrix
        P = np.array(camera_info_msg.p).reshape((3, 4))
        P_rotated = np.zeros_like(P)
        P_rotated[0, 0] = P[1, 1]  # fx_rot = fy
        P_rotated[1, 1] = P[0, 0]  # fy_rot = fx
        P_rotated[0, 2] = P[1, 2]  # cx_rot = cy
        P_rotated[1, 2] = P[0, 2]  # cy_rot = cx
        P_rotated[2, 2] = 1.0
        camera_info.p = P_rotated.flatten().tolist()

        # Copy distortion parameters
        camera_info.d = camera_info_msg.d
        camera_info.distortion_model = camera_info_msg.distortion_model

        # Publish the rotated camera info
        publisher.publish(camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRotateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
