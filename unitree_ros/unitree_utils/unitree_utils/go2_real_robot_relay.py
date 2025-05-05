import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, CameraInfo, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from hb40_commons.msg import RobotState, LegState
from unitree_go.msg import LowState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, CameraInfo, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from hb40_commons.msg import RobotState, LegState
from unitree_go.msg import LowState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class LowStateProcessor(Node):
    def __init__(self):
        super().__init__('lowstate_processor')
        
        # Subscribers
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.lowstate_callback,
            10)
        
        # self.color_camera_info_sub = self.create_subscription(
        #     CameraInfo, '/camera/color/camera_info', self.republish_color_camera_info, 10)
        # self.color_image_sub = self.create_subscription(
        #     Image, '/camera/color/image_raw', self.republish_color_image, 10)
        # self.depth_camera_info_sub = self.create_subscription(
        #     CameraInfo, '/camera/depth/camera_info', self.republish_depth_camera_info, 10)
        self.depth_image_sub = self.create_subscription(
            Image, '/d435i_camera/depth/image_rect_raw', self.republish_depth_image, 10)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/feet_contact_state', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/out', 10)
        
        # self.color_camera_info_pub = self.create_publisher(CameraInfo, '/d435i_camera/color/camera_info', 10)
        # self.color_image_pub = self.create_publisher(Image, '/d435i_camera/color/image_raw', 10)
        # self.depth_camera_info_pub = self.create_publisher(CameraInfo, '/d435i_camera/depth/camera_info', 10)
        # self.depth_image_pub = self.create_publisher(Image, '/d435i_camera/depth/image_rect_raw', 10)
        
        # Static Transform Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.stamp = None
        
        # Joint names
        self.joint_names = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        ]
        
        # Leg names
        self.leg_names = ["fr", "fl", "rr", "rl"]
        
    def publish_static_transforms(self, stamp):
        now = self.get_clock().now().to_msg()
        
        # if self.cam_info_header is None:
        #     return
        
        # if self.published_transforms:
        #     return
        
        # Transform from 'laser_frame' to 'body'
        laser_to_body = TransformStamped()
        # laser_to_body.header.stamp = self.cam_info_header.stamp
        # laser_to_body.header.stamp = self.robot_state_header.stamp
        laser_to_body.header.stamp = stamp
        laser_to_body.header.frame_id = 'base'
        laser_to_body.child_frame_id = 'laser_frame'
        laser_to_body.transform.translation.x = 0.290755
        laser_to_body.transform.translation.y = 0.000053
        laser_to_body.transform.translation.z = 0.068583
        laser_to_body.transform.rotation.x = 0.0
        laser_to_body.transform.rotation.y = 0.0
        laser_to_body.transform.rotation.z = 0.0
        laser_to_body.transform.rotation.w = 1.0
        
        body_to_imu = TransformStamped()
        # body_to_imu.header.stamp = self.cam_info_header.stamp
        # body_to_imu.header.stamp = self.robot_state_header.stamp
        body_to_imu.header.stamp = stamp
        body_to_imu.header.frame_id = 'base'
        body_to_imu.child_frame_id = 'imu_frame'
        body_to_imu.transform.translation.x = 0.0
        body_to_imu.transform.translation.y = 0.0
        body_to_imu.transform.translation.z = 0.0
        body_to_imu.transform.rotation.x = 0.0
        body_to_imu.transform.rotation.y = 0.0
        body_to_imu.transform.rotation.z = 0.0
        body_to_imu.transform.rotation.w = 1.0
        
        body_to_cam_optical_frame = TransformStamped()
        # body_to_cam_optical_frame.header.stamp = self.cam_info_header.stamp
        # body_to_cam_optical_frame.header.stamp = self.robot_state_header.stamp
        body_to_cam_optical_frame.header.stamp = stamp
        body_to_cam_optical_frame.header.frame_id = 'base'
        body_to_cam_optical_frame.child_frame_id = 'd435i_camera_color_optical_frame'
        body_to_cam_optical_frame.transform.translation.x = 0.286
        body_to_cam_optical_frame.transform.translation.y = 0.0
        body_to_cam_optical_frame.transform.translation.z = 0.036
        body_to_cam_optical_frame.transform.rotation.x = 0.500
        body_to_cam_optical_frame.transform.rotation.y = -0.500
        body_to_cam_optical_frame.transform.rotation.z = 0.500
        body_to_cam_optical_frame.transform.rotation.w = -0.500
        
        self.tf_broadcaster.sendTransform([laser_to_body, body_to_imu, body_to_cam_optical_frame])
    
    def lowstate_callback(self, msg):
        if self.stamp is None:
            return
        stamp = self.stamp
        self.publish_static_transforms(stamp)
        self.publish_joint_states(msg, stamp)
        self.publish_robot_state(msg, stamp)
        self.publish_imu(msg, stamp)
    
    def publish_joint_states(self, msg, stamp):
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = stamp
        
        joint_msg.name = self.joint_names
        joint_msg.position = [motor.q for motor in msg.motor_state][0:12]       # Only the first 12 motors, don't know what the other values are
        joint_msg.velocity = [motor.dq for motor in msg.motor_state][0:12]      # Only the first 12 motors, don't know what the other values are
        joint_msg.effort = [motor.tau_est for motor in msg.motor_state][0:12]   # Only the first 12 motors, don't know what the other values are
        
        self.joint_state_pub.publish(joint_msg)
    
    def publish_robot_state(self, msg, stamp):
        robot_msg = RobotState()
        robot_msg.header = Header()
        robot_msg.header.stamp = stamp
        
        for i in range(4):  # Order: FR, FL, RR, RL
            leg_msg = LegState()
            leg_msg.leg_name = self.leg_names[i]
            leg_msg.contact = bool(msg.foot_force[i] > 50)  # contact detection based on foot force
            leg_msg.foot_force_est = Vector3(x=0.0, y=0.0, z=float(msg.foot_force[i]))
            
            robot_msg.leg.append(leg_msg)
        
        self.robot_state_pub.publish(robot_msg)
    
    def publish_imu(self, msg, stamp):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = stamp
        
        # Extract roll, pitch, yaw from imu_state
        roll = float(msg.imu_state.rpy[0])
        pitch = float(msg.imu_state.rpy[1])
        yaw = float(msg.imu_state.rpy[2])

        # Convert Euler angles to quaternion
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Assign to imu_msg.orientation
        imu_msg.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        # imu_msg.orientation = Quaternion(
        #     x=float(msg.imu_state.quaternion[0]),
        #     y=float(msg.imu_state.quaternion[1]),
        #     z=float(msg.imu_state.quaternion[2]),
        #     w=float(msg.imu_state.quaternion[3])
        # )

        imu_msg.angular_velocity = Vector3(
            x=float(msg.imu_state.gyroscope[0]),
            y=float(msg.imu_state.gyroscope[1]),
            z=float(msg.imu_state.gyroscope[2])
        )
        
        imu_msg.linear_acceleration = Vector3(
            x=float(msg.imu_state.accelerometer[0]),
            y=float(msg.imu_state.accelerometer[1]),
            z=float(msg.imu_state.accelerometer[2])
        )
        
        self.imu_pub.publish(imu_msg)
    
    # def republish_color_camera_info(self, msg):
    #     self.color_camera_info_pub.publish(msg)
    
    # def republish_color_image(self, msg):
    #     self.color_image_pub.publish(msg)
    
    # def republish_depth_camera_info(self, msg):
    #     self.depth_camera_info_pub.publish(msg)
    
    def republish_depth_image(self, msg):
        self.stamp = msg.header.stamp

def main(args=None):
    rclpy.init(args=args)
    node = LowStateProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







# class LowStateProcessor(Node):
#     def __init__(self):
#         super().__init__('lowstate_processor')
        
#         # Declare parameters for image publish frequencies
#         self.declare_parameter('color_hz', 10.0)
#         self.declare_parameter('depth_hz', 50.0)
#         self.color_hz = self.get_parameter('color_hz').get_parameter_value().double_value
#         self.depth_hz = self.get_parameter('depth_hz').get_parameter_value().double_value
        
#         # Subscribers for low state and images
#         self.lowstate_sub = self.create_subscription(
#             LowState,
#             '/lowstate',
#             self.lowstate_callback,
#             10)
        
#         self.color_image_sub = self.create_subscription(
#             Image, '/camera/color/image_raw', self.color_image_callback, 10)
#         self.depth_image_sub = self.create_subscription(
#             Image, '/camera/depth/image_rect_raw', self.depth_image_callback, 10)
        
#         # Publishers for low state related topics
#         self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
#         self.robot_state_pub = self.create_publisher(RobotState, '/feet_contact_state', 10)
#         self.imu_pub = self.create_publisher(Imu, '/imu/out', 10)
        
#         # Publishers for filtered images (publish on separate topics to avoid loops)
#         self.color_image_pub = self.create_publisher(Image, '/d435i_camera/color/image_raw', 10)
#         self.depth_image_pub = self.create_publisher(Image, '/d435i_camera/depth/image_rect_raw', 10)
        
#         # Timers for publishing the latest image at fixed frequencies
#         self.color_timer = self.create_timer(1.0 / self.color_hz, self.publish_color_image)
#         self.depth_timer = self.create_timer(1.0 / self.depth_hz, self.publish_depth_image)
        
#         # Static Transform Broadcaster (as before)
#         self.tf_broadcaster = StaticTransformBroadcaster(self)
#         self.stamp = None
        
#         # Storage for the latest image messages
#         self.latest_color_msg = None
#         self.latest_depth_msg = None
        
#         # Joint and leg names
#         self.joint_names = [
#             "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
#             "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
#             "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
#             "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
#         ]
#         self.leg_names = ["fr", "fl", "rr", "rl"]
    
#     def publish_static_transforms(self, stamp):
#         now = self.get_clock().now().to_msg()
        
#         laser_to_body = TransformStamped()
#         laser_to_body.header.stamp = stamp
#         laser_to_body.header.frame_id = 'base'
#         laser_to_body.child_frame_id = 'laser_frame'
#         laser_to_body.transform.translation.x = 0.290755
#         laser_to_body.transform.translation.y = 0.000053
#         laser_to_body.transform.translation.z = 0.068583
#         laser_to_body.transform.rotation.x = 0.0
#         laser_to_body.transform.rotation.y = 0.0
#         laser_to_body.transform.rotation.z = 0.0
#         laser_to_body.transform.rotation.w = 1.0
        
#         body_to_imu = TransformStamped()
#         body_to_imu.header.stamp = stamp
#         body_to_imu.header.frame_id = 'base'
#         body_to_imu.child_frame_id = 'imu_frame'
#         body_to_imu.transform.translation.x = 0.0
#         body_to_imu.transform.translation.y = 0.0
#         body_to_imu.transform.translation.z = 0.0
#         body_to_imu.transform.rotation.x = 0.0
#         body_to_imu.transform.rotation.y = 0.0
#         body_to_imu.transform.rotation.z = 0.0
#         body_to_imu.transform.rotation.w = 1.0
        
#         body_to_cam_optical_frame = TransformStamped()
#         body_to_cam_optical_frame.header.stamp = stamp
#         body_to_cam_optical_frame.header.frame_id = 'base'
#         body_to_cam_optical_frame.child_frame_id = 'd435i_camera_color_optical_frame'
#         body_to_cam_optical_frame.transform.translation.x = 0.286
#         body_to_cam_optical_frame.transform.translation.y = 0.0
#         body_to_cam_optical_frame.transform.translation.z = 0.036
#         body_to_cam_optical_frame.transform.rotation.x = 0.500
#         body_to_cam_optical_frame.transform.rotation.y = -0.500
#         body_to_cam_optical_frame.transform.rotation.z = 0.500
#         body_to_cam_optical_frame.transform.rotation.w = -0.500
        
#         self.tf_broadcaster.sendTransform([laser_to_body, body_to_imu, body_to_cam_optical_frame])
    
#     def lowstate_callback(self, msg):
#         # Use the stamp from the lowstate message (or any other source) to drive transforms and other publishing.
#         if self.stamp is None:
#             return
#         stamp = self.stamp
#         self.publish_static_transforms(stamp)
#         self.publish_joint_states(msg, stamp)
#         self.publish_robot_state(msg, stamp)
#         self.publish_imu(msg, stamp)
    
#     def publish_joint_states(self, msg, stamp):
#         joint_msg = JointState()
#         joint_msg.header = Header()
#         joint_msg.header.stamp = stamp
        
#         joint_msg.name = self.joint_names
#         joint_msg.position = [motor.q for motor in msg.motor_state][0:12]
#         joint_msg.velocity = [motor.dq for motor in msg.motor_state][0:12]
#         joint_msg.effort = [motor.tau_est for motor in msg.motor_state][0:12]
        
#         self.joint_state_pub.publish(joint_msg)
    
#     def publish_robot_state(self, msg, stamp):
#         robot_msg = RobotState()
#         robot_msg.header = Header()
#         robot_msg.header.stamp = stamp
        
#         for i in range(4):
#             leg_msg = LegState()
#             leg_msg.leg_name = self.leg_names[i]
#             leg_msg.contact = bool(msg.foot_force[i] > 40)
#             leg_msg.foot_force_est = Vector3(x=0.0, y=0.0, z=float(msg.foot_force[i]))
#             robot_msg.leg.append(leg_msg)
        
#         self.robot_state_pub.publish(robot_msg)
    
#     def publish_imu(self, msg, stamp):
#         imu_msg = Imu()
#         imu_msg.header = Header()
#         imu_msg.header.stamp = stamp
        
#         roll = float(msg.imu_state.rpy[0])
#         pitch = float(msg.imu_state.rpy[1])
#         yaw = float(msg.imu_state.rpy[2])
#         quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
#         imu_msg.orientation = Quaternion(
#             x=quaternion[0],
#             y=quaternion[1],
#             z=quaternion[2],
#             w=quaternion[3]
#         )
        
#         imu_msg.angular_velocity = Vector3(
#             x=float(msg.imu_state.gyroscope[0]),
#             y=float(msg.imu_state.gyroscope[1]),
#             z=float(msg.imu_state.gyroscope[2])
#         )
        
#         imu_msg.linear_acceleration = Vector3(
#             x=float(msg.imu_state.accelerometer[0]),
#             y=float(msg.imu_state.accelerometer[1]),
#             z=float(msg.imu_state.accelerometer[2])
#         )
        
#         self.imu_pub.publish(imu_msg)
    
#     # Callbacks to store the latest image messages
#     def color_image_callback(self, msg):
#         self.latest_color_msg = msg

#     def depth_image_callback(self, msg):
#         self.stamp = msg.header.stamp
#         self.latest_depth_msg = msg

#     # Timer callbacks to publish the stored image messages at a fixed rate
#     def publish_color_image(self):
#         if self.latest_color_msg is not None:
#             # Optionally update the timestamp to the current time
#             # self.latest_color_msg.header.stamp = self.get_clock().now().to_msg()
#             self.color_image_pub.publish(self.latest_color_msg)

#     def publish_depth_image(self):
#         if self.latest_depth_msg is not None:
#             # self.latest_depth_msg.header.stamp = self.get_clock().now().to_msg()
#             self.depth_image_pub.publish(self.latest_depth_msg)
    
# def main(args=None):
#     rclpy.init(args=args)
#     node = LowStateProcessor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
