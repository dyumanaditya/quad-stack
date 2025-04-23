from nav_msgs.msg import Odometry
from hb40_commons.msg import BridgeData
from hb40_commons.msg import RobotState, LegState
from geometry_msgs.msg import Vector3
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import os
import pinocchio as pino
from scipy.spatial.transform import Rotation as R
import numpy as np
import rclpy
from rclpy.node import Node
from quadstack_contact.contact_detection import ContactDetector
from collections import deque

def compute_twist(pose_cur, pose_prev, dt):
    pos_cur, quat_cur = pose_cur
    pos_prev, quat_prev = pose_prev
    
    rot_cur = R.from_quat(quat_cur, scalar_first=False).as_matrix()
    rot_prev = R.from_quat(quat_prev, scalar_first=False).as_matrix()
    
    pose_cur_ = pino.SE3(rot_cur, pos_cur)
    pose_prev_ = pino.SE3(rot_prev, pos_prev)
    
    pose_diff = pose_prev_.actInv(pose_cur_)
    twist_fd = pino.log6(pose_diff) / dt

    return twist_fd.linear, twist_fd.angular

def map_ros_pin(joint_data_ros, joint_names_pin, joint_names_ros):
    # Map the joint data from ROS to Pinocchio
    joint_data_pin = []
    for joint_name in joint_names_pin:
        idx = joint_names_ros.index(joint_name)
        joint_data_pin_ = joint_data_ros[idx]
        joint_data_pin.append(joint_data_pin_)
    joint_data_pin = np.array(joint_data_pin)
    return joint_data_pin

def gen_pino_input(joint_pos, joint_vel, motor_torque, odom_pos, odom_ori, odom_lin_vel, odom_ang_vel, joint_names_pin, joint_names_ros):
    pino_pos = map_ros_pin(joint_pos, joint_names_pin, joint_names_ros)
    pino_vel = map_ros_pin(joint_vel, joint_names_pin, joint_names_ros)
    pino_tau = map_ros_pin(motor_torque, joint_names_pin, joint_names_ros)
    
    q = np.hstack([odom_pos, odom_ori, pino_pos])
    v = np.hstack([odom_lin_vel, odom_ang_vel, pino_vel])
    tau = pino_tau
    return q, v, tau

# define the subscriber to the /hb40/bridge_data and /odom topics
class ContactDetectorNode(Node):
    def __init__(self):
        super().__init__('ContactDetectorNode')
        self.declare_parameter('robot', 'silver_badger')
        self.robot = self.get_parameter('robot').value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Matches publisher
            durability=QoSDurabilityPolicy.VOLATILE,  # Matches publisher
            depth=10  # Depth is unknown, so we set a reasonable value
        )
        
        # Subscribe to the /joint_states and /odom topics
        self.bridge_data_subscriber = self.create_subscription(
            BridgeData,
            '/hb40/bridge_data',
            self.bridge_data_callback,
            qos_profile
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for the contact state
        self.contact_state_publisher = self.create_publisher(
            RobotState,
            '/estimated_contact_state',
            10
        )
        
        # Timer to periodically check for contact
        self.timer = self.create_timer(1 / 500, self.check_contact)
        
        # Initialize buffer for odometry data
        self.pos_prev = None
        self.ori_prev = None
        self.pos_buffer = deque(maxlen=10)
        self.ori_buffer = deque(maxlen=10)
        self.odom_timestamp = deque(maxlen=10)
        
        # Initilize buffer for joint states
        self.joint_pos = None
        self.joint_vel = None
        self.joint_tau = None
        self.joint_pos_buffer = deque(maxlen=50)
        self.joint_vel_buffer = deque(maxlen=50)
        self.joint_tau_buffer = deque(maxlen=50)
        self.joint_states_timestamp = deque(maxlen=50)
        
        odom_freq = 100 # 120
        self.dt = 1 / odom_freq
        self.init_odom_values()
        self._init_pino_model_data()
        self.contact_detector = ContactDetector(alg="mixing", pino_model=self.pino_model, pino_data=self.pino_data, nv=self.pino_model.nv, freq=odom_freq)
        
    def _init_pino_model_data(self):
        # Determine the URDF path based on the robot
        sb_description_pkg_share = get_package_share_directory('silver_badger_description')
        hb_description_pkg_share = get_package_share_directory('honey_badger_description')
        a1_description_pkg_share = get_package_share_directory('a1_description')
        go1_description_pkg_share = get_package_share_directory('go1_description')
        go2_description_pkg_share = get_package_share_directory('go2_description')
        
        urdf_paths = {
            'silver_badger': os.path.join(sb_description_pkg_share, 'urdf', 'silver_badger.urdf'),
            'honey_badger': os.path.join(hb_description_pkg_share, 'urdf', 'honey_badger.urdf'),
            'a1': os.path.join(a1_description_pkg_share, 'urdf', 'a1.urdf'),
            'go1': os.path.join(go1_description_pkg_share, 'urdf', 'go1.urdf'),
            'go2': os.path.join(go2_description_pkg_share, 'urdf', 'go2.urdf'),
        }
        urdf_path = urdf_paths[self.robot]
        # urdf_path = "/home/junninghuang/ros_ws/src/quad-stack/robots/silver_badger_description/urdf/silver_badger.urdf"
        
        # Load the robot model
        self.pino_model = pino.buildModelFromUrdf(urdf_path, pino.JointModelFreeFlyer())

        # Ros log info
        self.get_logger().info(f"Loaded robot model from {urdf_path}")
        self.get_logger().info(f"Robot ndof {self.pino_model.nv}")
        
        self.pino_data = self.pino_model.createData()
        self.foot_names = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
        self.joint_names_ros = ["fr_j0", "fr_j1", "fr_j2", "fl_j0", "fl_j1", "fl_j2", "rl_j0", "rl_j1", "rl_j2", "rr_j0", "rr_j1", "rr_j2", "sp_j0"]
        self.joint_names_pin = self.pino_model.names.tolist()[2: ]
        
    def bridge_data_callback(self, msg):
        # Process joint states data
        self.joint_states = msg
        
        # Retrieve joint states as numpy array
        self.joint_pos = np.array(msg.joint_position, dtype=np.float64)
        self.joint_vel = np.array(msg.joint_velocity, dtype=np.float64)
        self.joint_tau = np.array(msg.joint_effort, dtype=np.float64)
        
        # Save the data and timestamp in a buffer
        self.joint_pos_buffer.append(self.joint_pos)
        self.joint_vel_buffer.append(self.joint_vel)
        self.joint_tau_buffer.append(self.joint_tau)
        timestamp = self.get_clock().now().to_msg()
        self.joint_states_timestamp.append(timestamp)
        
    def odom_callback(self, msg):
        # Process odometry data
        self.odom = msg
        
        # Retrieve odometry data
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float64)
        self.ori = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w], dtype=np.float64)
        
        # Compute the twists
        if self.pos_prev is None or self.ori_prev is None:
            self.lin_vel = np.zeros(3)
            self.ang_vel = np.zeros(3)
        else:
            pose_cur = [self.pos, self.ori]
            pose_prev = [self.pos_prev, self.ori_prev]
            self.lin_vel, self.ang_vel = compute_twist(pose_cur, pose_prev, self.dt)
        
        # Update the previous pose
        self.pos_prev = self.pos
        self.ori_prev = self.ori
        
        # Update the buffer and timestamp buffer
        self.pos_buffer.append(self.pos)
        self.ori_buffer.append(self.ori)
        timestamp = self.get_clock().now().to_msg()
        self.odom_timestamp.append(timestamp)
        
    def init_odom_values(self):
         # Retrieve odometry data
        self.pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.ori = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        
        # Compute the twists
        if self.pos_prev is None or self.ori_prev is None:
            self.lin_vel = np.zeros(3)
            self.ang_vel = np.zeros(3)
        else:
            pose_cur = [self.pos, self.ori]
            pose_prev = [self.pos_prev, self.ori_prev]
            self.lin_vel, self.ang_vel = compute_twist(pose_cur, pose_prev, self.dt)
        
        # Update the previous pose
        self.pos_prev = self.pos
        self.ori_prev = self.ori
        
    def check_input(self, cur_timestamp):
        # Search in the joint states buffer for the closest timestamp
        closest_idx = None
        closest_diff = float('inf')
        for i, timestamp in enumerate(self.joint_states_timestamp):
            diff = abs((cur_timestamp.sec + cur_timestamp.nanosec * 1e-9) - (timestamp.sec + timestamp.nanosec * 1e-9))
            if diff < closest_diff:
                closest_diff = diff
                closest_idx = i
        if closest_idx is None:
            self.get_logger().warn("No joint states data available")
            return
        joint_pos = self.joint_pos_buffer[closest_idx]
        joint_vel = self.joint_vel_buffer[closest_idx]
        joint_tau = self.joint_tau_buffer[closest_idx]
        
        # Search in the odom buffer for the closest timestamp
        closest_idx = None
        closest_diff = float('inf')
        for i, timestamp in enumerate(self.odom_timestamp):
            diff = abs((cur_timestamp.sec + cur_timestamp.nanosec * 1e-9) - (timestamp.sec + timestamp.nanosec * 1e-9))
            if diff < closest_diff:
                closest_diff = diff
                closest_idx = i
        if closest_idx is None:
            self.get_logger().warn("No odometry data available")
            return
        pos = self.pos_buffer[closest_idx]
        ori = self.ori_buffer[closest_idx]
        pos_prev = self.pos_buffer[closest_idx - 1] if closest_idx > 0 else self.pos_buffer[closest_idx]
        ori_prev = self.ori_buffer[closest_idx - 1] if closest_idx > 0 else self.ori_buffer[closest_idx]
        
        # Compute the twists
        lin_vel, ang_vel = compute_twist([pos, ori], [pos_prev, ori_prev], self.dt)
        return joint_pos, joint_vel, joint_tau, pos, ori, lin_vel, ang_vel
        
    def check_contact(self):
        if self.joint_pos is None or self.joint_vel is None or self.joint_tau is None:
            return
        if self.pos_prev is None or self.ori_prev is None:
            return
        
        cur_timestamp = self.get_clock().now().to_msg()
        
        # # Get the closest joint states and odometry data
        # joint_pos, joint_vel, joint_tau, pos, ori, lin_vel, ang_vel = self.check_input(cur_timestamp)
        # q, v, tau = gen_pino_input(joint_pos, joint_vel, joint_tau, pos, ori, lin_vel, ang_vel, self.joint_names_pin, self.joint_names_ros)
        
        # Generate the input for the contact detector
        q, v, tau = gen_pino_input(self.joint_pos, self.joint_vel, self.joint_tau, self.pos, self.ori, self.lin_vel, self.ang_vel, self.joint_names_pin, self.joint_names_ros)
        est_f, est_f_filtered, contact_states = self.contact_detector.apply_contact_detection(q, v, tau)
        
        # Get the peusdo ground truth of the contact states
        feet_pos, feet_pos_filtered, contact_states_gt = self.contact_detector.apply_contact_detection_gt(q)
        
        # Create the RobotState message
        robot_state_msg = RobotState()
        robot_state_msg.header.stamp = cur_timestamp
        robot_state_msg.header.frame_id = "base_link"
        
        # Fill in the robot state message
        robot_state_msg.joint_pos = [float(q[i]) for i in range(len(q))]
        robot_state_msg.joint_vel = [float(v[i]) for i in range(len(v))]
        robot_state_msg.joint_torque = [float(tau[i]) for i in range(len(tau))]
        
        # Fill in the contact state message
        for i in range(len(self.foot_names)):
            foot_name = self.foot_names[i]
            contact_state = contact_states[foot_name]

            leg_msg = LegState()
            leg_msg.leg_name = foot_name
            leg_msg.contact = bool(contact_state)
            
            contact_state_gt = contact_states_gt[foot_name]
            leg_msg.contact_gt = bool(contact_state_gt)
            
            est_f_cur_foot = est_f[i*3: i*3 + 3]
            est_force_msg = Vector3()
            est_force_msg.x = float(est_f_cur_foot[0])
            est_force_msg.y = float(est_f_cur_foot[1])
            est_force_msg.z = float(est_f_cur_foot[2])
            leg_msg.foot_force_est = est_force_msg

            est_f_filtered_cur_foot = est_f_filtered[i*3: i*3 + 3]
            est_force_filtered_msg = Vector3()
            est_force_filtered_msg.x = float(est_f_filtered_cur_foot[0])
            est_force_filtered_msg.y = float(est_f_filtered_cur_foot[1])
            est_force_filtered_msg.z = float(est_f_filtered_cur_foot[2])
            leg_msg.foot_force_est_filter = est_force_filtered_msg

            foot_pos = feet_pos[foot_name]
            foot_pos_msg = Vector3()
            foot_pos_msg.x = float(foot_pos[0])
            foot_pos_msg.y = float(foot_pos[1])
            foot_pos_msg.z = float(foot_pos[2])
            leg_msg.foot_pos = foot_pos_msg
            
            foot_pos_filtered = feet_pos_filtered[foot_name]
            foot_pos_filtered_msg = Vector3()   
            foot_pos_filtered_msg.x = float(foot_pos_filtered[0])
            foot_pos_filtered_msg.y = float(foot_pos_filtered[1])
            foot_pos_filtered_msg.z = float(foot_pos_filtered[2])
            leg_msg.foot_pos_filter = foot_pos_filtered_msg

            robot_state_msg.leg.append(leg_msg)
        
        # Publish the contact state message
        self.contact_state_publisher.publish(robot_state_msg)
    
def main(args=None):
    rclpy.init(args=args)
    contact_detector_node = ContactDetectorNode()
    rclpy.spin(contact_detector_node)
    contact_detector_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()