import rclpy
from rclpy.node import Node
from hb40_commons.msg import BridgeData, RobotState
from sensor_msgs.msg import JointState, Imu, CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class BridgeDataProcessor(Node):
    def __init__(self):
        super().__init__('bridge_data_processor')
        
        # Keep track of the bridge data header
        self.robot_state_header = None
        self.cam_info_header = None
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Matches publisher
            durability=QoSDurabilityPolicy.VOLATILE,  # Matches publisher
            depth=10  # Depth is unknown, so we set a reasonable value
        )
        
        # Subscriber to BridgeData topic
        self.subscription = self.create_subscription(
            BridgeData,
            '/hb40/bridge_data',
            self.bridge_data_callback,
            qos_profile)
        
        # Subscribe to camera info to get the time header
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/hb40/robot_state',
            self.robot_state_callback,
            qos_profile)
        
        # Subscribe to camera info to get the time header
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/d435i_camera/depth/camera_info',
            self.cam_info_callback,
            10)
        
        # Publishers for JointStates and IMU data
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/out', 10)
        
        # Static Transform Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        # self.tf_broadcaster = TransformBroadcaster(self)
        
        # timer to publish static transforms
        # self.publish_static_transforms()
        # self.create_timer(0.002, self.publish_static_transforms)
        self.published_transforms = False
        
    
    def publish_static_transforms(self, stamp):
        now = self.get_clock().now().to_msg()
        
        # if self.cam_info_header is None:
        #     return
        
        if self.published_transforms:
            return
        
        self.published_transforms = True
        
        # self.published_transforms = True
        # stamp = self.cam_info_header.stamp
        
        # Transform from 'base_link' to 'hb40/body'
        base_to_body = TransformStamped()
        # base_to_body.header.stamp = self.cam_info_header.stamp
        # base_to_body.header.stamp = self.robot_state_header.stamp
        base_to_body.header.stamp = stamp
        base_to_body.header.frame_id = 'base_link'
        base_to_body.child_frame_id = 'hb40/base_link'
        base_to_body.transform.translation.x = 0.0
        base_to_body.transform.translation.y = 0.0
        base_to_body.transform.translation.z = 0.0
        base_to_body.transform.rotation.x = 0.0
        base_to_body.transform.rotation.y = 0.0
        base_to_body.transform.rotation.z = 0.0
        base_to_body.transform.rotation.w = 1.0
        
        # Transform from 'laser_frame' to 'body'
        laser_to_body = TransformStamped()
        # laser_to_body.header.stamp = self.cam_info_header.stamp
        # laser_to_body.header.stamp = self.robot_state_header.stamp
        laser_to_body.header.stamp = stamp
        laser_to_body.header.frame_id = 'hb40/base_link'
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
        body_to_imu.header.frame_id = 'hb40/base_link'
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
        body_to_cam_optical_frame.header.frame_id = 'base_link'
        body_to_cam_optical_frame.child_frame_id = 'd435i_camera_color_optical_frame'
        body_to_cam_optical_frame.transform.translation.x = 0.286
        body_to_cam_optical_frame.transform.translation.y = 0.0
        body_to_cam_optical_frame.transform.translation.z = 0.036
        body_to_cam_optical_frame.transform.rotation.x = 0.707
        body_to_cam_optical_frame.transform.rotation.y = 0.0
        body_to_cam_optical_frame.transform.rotation.z = 0.707
        body_to_cam_optical_frame.transform.rotation.w = 0.0
        
        self.tf_broadcaster.sendTransform([base_to_body, laser_to_body, body_to_imu, body_to_cam_optical_frame])
    
    def bridge_data_callback(self, msg: BridgeData):
        if self.robot_state_header is None:
            return
        
        stamp = self.robot_state_header.stamp
        
        self.publish_static_transforms(stamp)
        
        # Process Joint State Data
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = stamp
        joint_state_msg.name = msg.joint_name
        joint_state_msg.position = list(msg.joint_position)
        joint_state_msg.velocity = list(msg.joint_velocity)
        joint_state_msg.effort = list(msg.joint_effort)
        
        self.joint_state_publisher.publish(joint_state_msg)
        
        # Process IMU Data
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_frame'
        imu_msg.orientation = msg.orientation
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration
        
        self.imu_publisher.publish(imu_msg)
        
    def robot_state_callback(self, msg: RobotState):
        self.robot_state_header = msg.header
        
    def cam_info_callback(self, msg: CameraInfo):
        self.cam_info_header = msg.header
        

def main(args=None):
    rclpy.init(args=args)
    node = BridgeDataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
