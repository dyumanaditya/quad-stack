#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import math

# For quaternion to Euler conversion.
import tf_transformations

# TF2
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time


class NavigateToPoses(Node):
    def __init__(self):
        super().__init__('navigate_to_poses')
        self.get_logger().info("Navigation Tester node starting...")

        # Action client for sending goal poses using the NavigateToPose action.
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher for goal_pose topic.
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF2 Buffer and Listener to obtain the final robot pose.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot and environment configuration.
        # self.robot = "silver_badger"
        self.robot = "go2"
        # self.env = "house"
        self.env = "warehouse"

        # List of five goal poses from your provided data.
        self.goals = []

        if self.env == "warehouse":
            # Pose 1
            pose1 = PoseStamped()
            pose1.header.frame_id = "map"
            pose1.pose.position.x = 0.8749198913574219
            pose1.pose.position.y = -6.274601936340332
            pose1.pose.position.z = 0.0
            pose1.pose.orientation.x = 0.0
            pose1.pose.orientation.y = 0.0
            pose1.pose.orientation.z = -0.6848786966675345
            pose1.pose.orientation.w = 0.7286571010090955

            # Pose 2
            pose2 = PoseStamped()
            pose2.header.frame_id = "map"
            pose2.pose.position.x = -3.843245506286621
            pose2.pose.position.y = -6.959385871887207
            pose2.pose.position.z = 0.0
            pose2.pose.orientation.x = 0.0
            pose2.pose.orientation.y = 0.0
            pose2.pose.orientation.z = -0.7171964831106268
            pose2.pose.orientation.w = 0.6968710100253478

            # Pose 3
            pose3 = PoseStamped()
            pose3.header.frame_id = "map"
            pose3.pose.position.x = 1.159440040588379
            pose3.pose.position.y = 5.573970317840576
            pose3.pose.position.z = 0.0
            pose3.pose.orientation.x = 0.0
            pose3.pose.orientation.y = 0.0
            pose3.pose.orientation.z = 0.7209753196051547
            pose3.pose.orientation.w = 0.6929607409660702

            # Pose 4
            pose4 = PoseStamped()
            pose4.header.frame_id = "map"
            pose4.pose.position.x = -3.776885986328125
            pose4.pose.position.y = 8.0328070640563965
            pose4.pose.position.z = 0.0
            pose4.pose.orientation.x = 0.0
            pose4.pose.orientation.y = 0.0
            pose4.pose.orientation.z = 0.7052376459019037
            pose4.pose.orientation.w = 0.7089709886890584

            # Pose 5
            pose5 = PoseStamped()
            pose5.header.frame_id = "map"
            pose5.pose.position.x = 0.3695840835571289
            pose5.pose.position.y = -0.6874237060546875
            pose5.pose.position.z = 0.0
            pose5.pose.orientation.x = 0.0
            pose5.pose.orientation.y = 0.0
            pose5.pose.orientation.z = 0.03371763975647777
            pose5.pose.orientation.w = 0.9994313987309246
        else:
           # Create goal 1 PoseStamped message
            pose1 = PoseStamped()
            pose1.header.frame_id = "map"
            pose1.pose.position.x = 6.501738262176514
            pose1.pose.position.y = -3.489607334136963
            pose1.pose.position.z = 0.0
            pose1.pose.orientation.x = 0.0
            pose1.pose.orientation.y = 0.0
            pose1.pose.orientation.z = -0.007392605132624851
            pose1.pose.orientation.w = 0.9999726743213302

            # Create goal 2 PoseStamped message
            pose2 = PoseStamped()
            pose2.header.frame_id = "map"
            pose2.pose.position.x = -7.08655071258545
            pose2.pose.position.y = -2.875117540359497
            pose2.pose.position.z = 0.0
            pose2.pose.orientation.x = 0.0
            pose2.pose.orientation.y = 0.0
            pose2.pose.orientation.z = 0.9999808527545784
            pose2.pose.orientation.w = 0.006188224642512183

            # Create goal 3 PoseStamped message
            pose3 = PoseStamped()
            pose3.header.frame_id = "map"
            pose3.pose.position.x = 1.5017441511154175
            pose3.pose.position.y = 2.5557174682617188
            pose3.pose.position.z = 0.0
            pose3.pose.orientation.x = 0.0
            pose3.pose.orientation.y = 0.0
            pose3.pose.orientation.z = 0.7046049097465081
            pose3.pose.orientation.w = 0.7095998317087703

            # Create goal 4 PoseStamped message
            pose4 = PoseStamped()
            pose4.header.frame_id = "map"
            pose4.pose.position.x = -7.035238265991211
            pose4.pose.position.y = -0.22752340137958527
            pose4.pose.position.z = 0.0
            pose4.pose.orientation.x = 0.0
            pose4.pose.orientation.y = 0.0
            pose4.pose.orientation.z = -0.9999064933522839
            pose4.pose.orientation.w = 0.013674960765541476

            # Create goal 5 PoseStamped message
            pose5 = PoseStamped()
            pose5.header.frame_id = "map"
            pose5.pose.position.x = 5.006673336029053
            pose5.pose.position.y = 0.8600801825523376
            pose5.pose.position.z = 0.0
            pose5.pose.orientation.x = 0.0
            pose5.pose.orientation.y = 0.0
            pose5.pose.orientation.z = 0.017143167362639187
            pose5.pose.orientation.w = 0.9998530451085182

        self.goals = [pose1, pose2, pose3, pose4, pose5]
        self.current_goal_index = 0

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Helper function to create a PoseStamped message in the 'map' frame."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        # Convert yaw to quaternion.
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        return pose_stamped

    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_pose = self.goals[self.current_goal_index]
            self.get_logger().info(f"Sending goal {self.current_goal_index + 1}: "
                                   f"x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}")

            # Publish the goal pose onto the '/goal_pose' topic.
            self.goal_pose_pub.publish(goal_pose)
            
            # Wait until the action server is available.
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Action server not available after waiting")
                rclpy.shutdown()
                return

            # Construct the goal message.
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose

            # Record the start time.
            start_time = time.time()

            # Send the goal asynchronously.
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(
                lambda future, start_time=start_time, goal_pose=goal_pose: self.goal_response_callback(
                    future, start_time, goal_pose))
        else:
            self.get_logger().info("Finished sending all goals.")
            rclpy.shutdown()

    def goal_response_callback(self, future, start_time, goal_pose):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            self.current_goal_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future, start_time=start_time, goal_pose=goal_pose: self.result_callback(
                future, start_time, goal_pose))

    def feedback_callback(self, feedback_msg):
        # Optionally log feedback info here.
        self.get_logger().debug("Received feedback")

    def result_callback(self, future, start_time, goal_pose):
        end_time = time.time()
        duration = end_time - start_time

        # Retrieve the robot's final pose via TF.
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link' if self.robot in ['silver_badger', 'honey_badger'] else 'base',
                Time(),
                timeout=Duration(seconds=1.0)
            )
            final_x = trans.transform.translation.x
            final_y = trans.transform.translation.y
            quat = trans.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            _, _, final_yaw = tf_transformations.euler_from_quaternion(quat_list)
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            final_x, final_y, final_yaw = 0.0, 0.0, 0.0

        # Extract the goal pose.
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        goal_quat = goal_pose.pose.orientation
        goal_quat_list = [goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w]
        _, _, goal_yaw = tf_transformations.euler_from_quaternion(goal_quat_list)

        error_x = goal_x - final_x
        error_y = goal_y - final_y
        error_yaw = goal_yaw - final_yaw
        # Normalize angular error to [-pi, pi].
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        self.get_logger().info(
            f"Goal {self.current_goal_index + 1} reached in {duration:.2f} seconds")
        self.get_logger().info(
            f"Error: Δx = {error_x:.2f}, Δy = {error_y:.2f}, Δyaw = {math.degrees(error_yaw):.2f} deg")

        self.current_goal_index += 1
        # Wait for a brief period before sending the next goal.
        time.sleep(2.0)
        self.send_next_goal()

    # This method sends the first goal when the node starts.
    def start_test(self):
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoses()
    node.start_test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation test interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
