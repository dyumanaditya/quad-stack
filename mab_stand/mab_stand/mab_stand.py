# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import numpy as np
# from hb40_commons.msg import JointCommand



# class MABStand(Node):

#     def __init__(self):
#         super().__init__('mab_stand')
        
#         # self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
#         self.joint_trajectory_publisher = self.create_publisher(JointCommand, '/hb40/joint_command', 10)
#         self.timer = self.create_timer(1.0, self.publish_trajectory_once)
#         self.already_published = False

#         # Define target joint positions for standing
#         self.target_positions = {
#             'fr_j0': -0.1,
#             'fr_j1': 0.8,
#             'fr_j2': -1.5,
#             'fl_j0': 0.1,
#             'fl_j1': -0.8,
#             'fl_j2': 1.5,
#             'rl_j0': -0.1,
#             'rl_j1': -1.0,
#             'rl_j2': 1.5,
#             'rr_j0': 0.1,
#             'rr_j1': 1.0,
#             'rr_j2': -1.5,
#             'sp_j0': 0.0
#         }

#         self.num_points = 50  # Number of points for interpolation
#         self.duration = 2.0   # Duration of the trajectory in seconds

#         self.joint_names = list(self.target_positions.keys())
#         self.start_positions = [0.0] * len(self.joint_names)

#     def interpolate_trajectory(self, start_positions, target_positions, num_points, duration):
#         times = np.linspace(0, duration, num_points)
#         trajectory_points = []
#         positions_set = []

#         for t in times:
#             positions = [
#                 start + (target - start) * (t / duration)
#                 for start, target in zip(start_positions, target_positions)
#             ]
#             positions_set.append(positions)

#             point = JointTrajectoryPoint()
#             point.positions = positions
#             point.time_from_start = rclpy.time.Duration(seconds=t).to_msg()
#             trajectory_points.append(point)

#         return trajectory_points, positions_set

#     def publish_trajectory_once(self):
#         if self.already_published:
#             return
        
#         # joint_trajectory_msg = JointTrajectory()
#         # joint_trajectory_msg.joint_names = self.joint_names

#         # target_positions = [self.target_positions[name] for name in self.joint_names]
#         # joint_trajectory_msg.points, _ = self.interpolate_trajectory(
#         #     self.start_positions, target_positions, self.num_points, self.duration
#         # )

#         # self.joint_trajectory_publisher.publish(joint_trajectory_msg)
#         # self.get_logger().info('Published joint trajectory to stand')
        
#         # self.already_published = True
#         # self.timer.cancel()


#         target_positions = [self.target_positions[name] for name in self.joint_names]
#         _, positions = self.interpolate_trajectory(
#             self.start_positions, target_positions, self.num_points, self.duration
#         )

#         # for pos_set in positions:
#         joint_trajectory_msg = JointCommand()
#         joint_trajectory_msg.name = self.joint_names
#         joint_trajectory_msg.t_pos = target_positions

#         self.joint_trajectory_publisher.publish(joint_trajectory_msg)
#         # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

#         self.get_logger().info('Published joint trajectory to stand')
        
#         # self.already_published = True
#         # self.timer.cancel()

# def main(args=None):
#     rclpy.init(args=args)
#     node = MABStand()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from hb40_commons.msg import JointCommand
import numpy as np
import time


class MABStand(Node):

    def __init__(self):
        super().__init__('mab_stand')
        self.publisher_ = self.create_publisher(JointCommand, '/hb40/joint_command', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.first_time = True

    def timer_callback(self):
        if self.first_time:
            self.first_time = False
            start_positions = [0.0] * 13
            target_positions = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5, 0.0]

            # interpolate trajectory
            num_points = 50
            duration = 2.0
            positions_set = self.interpolate_trajectory(start_positions, target_positions, num_points, duration)

            for positions in positions_set:
                msg = JointCommand()
                msg.t_pos = positions
                msg.kd = [0.5] * 13
                msg.kp = [20.0] * 13
                self.publisher_.publish(msg)
                time.sleep(duration / num_points)
        else:
            pass
            # msg = JointCommand()
            # msg.t_pos = [-0.1, 0.8, -1.5, 0.1, -0.8, 1.5, -0.1, -1.0, 1.5, 0.1, 1.0, -1.5, 0.0]
            # msg.kd = [0.5] * 13
            # msg.kp = [20.0] * 13
            # self.publisher_.publish(msg)

    def interpolate_trajectory(self, start_positions, target_positions, num_points, duration):
        times = np.linspace(0, duration, num_points)
        positions_set = []

        for t in times:
            positions = [
                start + (target - start) * (t / duration)
                for start, target in zip(start_positions, target_positions)
            ]
            positions_set.append(positions)

        return positions_set

def main(args=None):
    rclpy.init(args=args)
    node = MABStand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
