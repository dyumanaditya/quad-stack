import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations

class FrameStabilizer(Node):
    def __init__(self):
        super().__init__('frame_stabilizer')

        # Subscribers
        self.depth_sub = self.create_subscription(Image, '/depth_input', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_camera_info', self.camera_info_callback, 10)

        # Publishers
        self.stabilized_depth_pub = self.create_publisher(Image, '/depth_stabilized', 10)
        self.stabilized_camera_info_pub = self.create_publisher(CameraInfo, '/depth_stabilized_camera_info', 10)

        # CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Initialize roll and pitch
        self.roll = 0.0
        self.pitch = 0.0

        # Store the latest camera info
        self.camera_info = None

    def imu_callback(self, msg: Imu):
        # Extract orientation as a quaternion
        orientation_q = msg.orientation
        # Convert quaternion to roll, pitch, and yaw
        roll, pitch, _ = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Update roll and pitch
        self.roll = roll
        self.pitch = pitch

    def camera_info_callback(self, msg: CameraInfo):
        # Store the camera info message
        self.camera_info = msg

    def depth_callback(self, msg: Image):
        if self.camera_info is None:
            # If we don't have camera info yet, skip processing
            return

        # Convert the depth image to an OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Calculate rotation matrix based on pitch and roll
        height, width = depth_image.shape[:2]
        transformation_mat = self.get_transformation_matrix(self.roll, self.pitch, height, width)

        # rclpy.logging.get_logger('frame_stabilizer').info(f'Rotating Image by {self.roll}')


        # Stabilize the image using the rotation matrix
        stabilized_depth_image = cv2.warpAffine(depth_image, transformation_mat, (height, width), borderValue=0)

        # Convert the stabilized image back to a ROS Image message and publish it
        stabilized_msg = self.bridge.cv2_to_imgmsg(stabilized_depth_image, encoding='passthrough')
        stabilized_msg.header = msg.header

        self.stabilized_depth_pub.publish(stabilized_msg)

        # Adjust the camera info
        adjusted_camera_info = self.adjust_camera_info(self.camera_info, transformation_mat)
        self.stabilized_camera_info_pub.publish(adjusted_camera_info)

    def get_transformation_matrix(self, roll, pitch, height, width):
        # Convert roll and pitch to radians (if not already in radians)
        roll_rad = roll
        pitch_rad = pitch
        

        # Roll creates the rotation angle and pitch creates the translation
        rotation_angle = -np.rad2deg(roll_rad)
        translation_x = 0
        translation_y = 0

        # Calculate translation based on pitch and camera parameters
        if self.camera_info:
            fx = self.camera_info.k[0]  # Focal length in x
            fy = self.camera_info.k[4]  # Focal length in y
            translation_y = np.tan(pitch_rad) * fy  # Pitch translation in pixels
        else:
            translation_y = 0

        rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), rotation_angle, 1)
        transformation_matrix = np.zeros((2, 3), np.float32)
        transformation_matrix[:, :2] = rotation_matrix[:, :2]
        transformation_matrix[:, 2] = [translation_x, -translation_y]

        return transformation_matrix

    def adjust_camera_info(self, camera_info, transformation_mat):
        # Clone the camera info to modify it
        adjusted_camera_info = CameraInfo()
        adjusted_camera_info.header = camera_info.header
        adjusted_camera_info.height = camera_info.height
        adjusted_camera_info.width = camera_info.width
        adjusted_camera_info.distortion_model = camera_info.distortion_model
        adjusted_camera_info.d = camera_info.d
        adjusted_camera_info.r = camera_info.r
        adjusted_camera_info.p = camera_info.p

        # Extract the 2x2 rotation part and the translation part of the affine matrix
        rotation_part = transformation_mat[:, :2]
        translation_part = transformation_mat[:, 2]

        # Adjust the intrinsic matrix K
        K = np.array(camera_info.k).reshape(3, 3)

        # Adjust the principal point (cx, cy) using the translation part of the affine matrix
        cx = K[0, 2]
        cy = K[1, 2]
        new_cx = rotation_part[0, 0] * cx + rotation_part[0, 1] * cy + translation_part[0]
        new_cy = rotation_part[1, 0] * cx + rotation_part[1, 1] * cy + translation_part[1]

        # Update the camera info with the new principal point
        adjusted_camera_info.k = K.flatten().tolist()
        adjusted_camera_info.k[2] = new_cx  # Update cx
        adjusted_camera_info.k[5] = new_cy  # Update cy

        # Projection matrix P: Adjust the principal point similarly in the P matrix
        P = np.array(camera_info.p).reshape(3, 4)
        P[0, 2] = new_cx  # Update cx in P matrix
        P[1, 2] = new_cy  # Update cy in P matrix
        adjusted_camera_info.p = P.flatten().tolist()

        return adjusted_camera_info


def main(args=None):
    rclpy.init(args=args)
    node = FrameStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
