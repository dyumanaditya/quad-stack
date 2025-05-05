#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
# replace `your_package` with the actual package name providing RobotState
from hb40_commons.msg import RobotState
import tf_transformations
import matplotlib.pyplot as plt

class VelocityComparisonNode(Node):
    def __init__(self):
        super().__init__('velocity_comparison_node')
        # Time reference
        self.initial_time = None

        # Time arrays for velocities
        self.t_base = []
        self.t_opti = []

        # Storage for base-frame velocities
        keys = ['linear_x','linear_y','linear_z','angular_x','angular_y','angular_z']
        self.base_v = {k: [] for k in keys}
        self.opti_v = {k: [] for k in keys}

        # Last optitrack pose/time for computing velocities
        self.last_opt_pose = None
        self.last_opt_time = None

        # Time arrays for contact state
        self.t_contact_raw = []   # raw ROS timestamps in seconds
        self.t_contact     = []   # relative time since first contact message

        # Per-leg contact storage
        self.contact    = {}  # estimated
        self.contact_gt = {}  # ground truth
        self.leg_names  = []

        # Subscriptions
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/base_lin_vel',
            self.base_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/optitrack/odom',
            self.opti_callback,
            10
        )
        self.create_subscription(
            RobotState,
            '/feet_contact_state',
            self.contact_callback,
            10
        )

    def base_callback(self, msg: TwistWithCovarianceStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.initial_time is None:
            self.initial_time = t
        rel_t = t - self.initial_time

        twist = msg.twist.twist
        self.t_base.append(rel_t)
        self.base_v['linear_x'].append(twist.linear.x)
        self.base_v['linear_y'].append(twist.linear.y)
        self.base_v['linear_z'].append(twist.linear.z)
        self.base_v['angular_x'].append(twist.angular.x)
        self.base_v['angular_y'].append(twist.angular.y)
        self.base_v['angular_z'].append(twist.angular.z)

    def opti_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.initial_time is None:
            self.initial_time = t
        rel_t = t - self.initial_time

        pose = msg.pose.pose
        if self.last_opt_pose is None:
            self.last_opt_pose = pose
            self.last_opt_time = t
            return

        dt = t - self.last_opt_time
        if dt <= 0:
            return

        # World-frame linear velocity
        dx = pose.position.x - self.last_opt_pose.position.x
        dy = pose.position.y - self.last_opt_pose.position.y
        dz = pose.position.z - self.last_opt_pose.position.z
        vx_w, vy_w, vz_w = dx/dt, dy/dt, dz/dt

        # World-frame angular velocity
        q_prev = [
            self.last_opt_pose.orientation.x,
            self.last_opt_pose.orientation.y,
            self.last_opt_pose.orientation.z,
            self.last_opt_pose.orientation.w
        ]
        q_curr = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        q_delta = tf_transformations.quaternion_multiply(
            q_curr,
            tf_transformations.quaternion_inverse(q_prev)
        )
        angle = 2 * math.acos(max(-1.0, min(1.0, q_delta[3])))
        if abs(angle) < 1e-6:
            wx_w = wy_w = wz_w = 0.0
        else:
            axis = [q_delta[i]/math.sin(angle/2) for i in range(3)]
            wx_w, wy_w, wz_w = [axis[i] * angle / dt for i in range(3)]

        self.last_opt_pose = pose
        self.last_opt_time = t

        # Rotate into base frame
        q = pose.orientation
        R_bw = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[0:3,0:3]
        R_wb = R_bw.T
        v_local = R_wb.dot([vx_w, vy_w, vz_w])
        w_local = R_wb.dot([wx_w, wy_w, wz_w])

        self.t_opti.append(rel_t)
        for key, val in zip(
            ['linear_x','linear_y','linear_z','angular_x','angular_y','angular_z'],
            list(v_local) + list(w_local)
        ):
            self.opti_v[key].append(val)

    def contact_callback(self, msg: RobotState):
        # Raw timestamp
        t_raw = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t_contact_raw.append(t_raw)

        # Initialize initial_time on first contact message
        if self.initial_time is None:
            self.initial_time = t_raw
        rel_t = t_raw - self.initial_time
        self.t_contact.append(rel_t)

        # On first message, capture leg names and init lists
        if not self.leg_names:
            self.leg_names = [leg.leg_name for leg in msg.leg]
            for name in self.leg_names:
                self.contact[name] = []
                self.contact_gt[name] = []

        # Append contact flags
        for leg in msg.leg:
            self.contact[leg.leg_name].append(1 if leg.contact else 0)
            self.contact_gt[leg.leg_name].append(1 if leg.contact_gt else 0)

    def plot_results(self):
        # --- velocity comparisons ---
        labels = [
            ('linear_x','Linear X Velocity (m/s)'),
            ('linear_y','Linear Y Velocity (m/s)'),
            ('linear_z','Linear Z Velocity (m/s)'),
            ('angular_x','Angular X Velocity (rad/s)'),
            ('angular_y','Angular Y Velocity (rad/s)'),
            ('angular_z','Angular Z Velocity (rad/s)')
        ]
        for key, ylabel in labels:
            plt.figure(figsize=(10,6))
            plt.plot(self.t_opti,
                     self.opti_v[key],
                     color='C1',
                     label='/optitrack/odom',
                     zorder=1)
            plt.plot(self.t_base,
                     self.base_v[key],
                     color='C0',
                     label='/base_lin_vel',
                     zorder=2)
            plt.xlabel('Time (s)')
            plt.ylabel(ylabel)
            plt.title(f'Comparison of {key.replace("_"," ")}')
            plt.legend()
            plt.grid(True)
            plt.ylim(-5, 5)

        # --- bucket contacts into 0.5 s bins ---
        bin_width = 0.5
        if not self.t_contact:
            return  # no contact data to plot
        max_t = max(self.t_contact)
        bins = np.arange(0.0, max_t + bin_width, bin_width)
        bin_centers = bins[:-1] + bin_width/2

        contact_binned    = {name: [] for name in self.leg_names}
        contact_gt_binned = {name: [] for name in self.leg_names}

        bin_width = 0.5
        if not self.t_contact:
            return  # no contact data

        # convert to numpy arrays
        t_arr = np.array(self.t_contact)
        max_t = t_arr.max()
        # edges from 0 to max_t in steps of 0.5s
        bins = np.arange(0.0, max_t + bin_width, bin_width)
        # centers for plotting
        centers = bins[:-1] + bin_width/2
        n_bins = len(centers)

        # digitize gives you bin indices (1..len(bins)), subtract 1 for 0-based
        bin_idx = np.digitize(t_arr, bins) - 1
        # clamp to valid range
        bin_idx = np.clip(bin_idx, 0, n_bins-1)

        # prepare storage
        contact_binned    = {name: [] for name in self.leg_names}
        contact_gt_binned = {name: [] for name in self.leg_names}

        # for each leg, find which bins saw any contact==1
        for name in self.leg_names:
            est = np.array(self.contact[name], dtype=bool)
            gt  = np.array(self.contact_gt[name], dtype=bool)

            # only consider timestamps where est==True
            est_bins = bin_idx[est]
            gt_bins  = bin_idx[gt]

            # unique bin indices
            est_unique = np.unique(est_bins)
            gt_unique  = np.unique(gt_bins)

            # map to center times
            contact_binned[name]    = centers[est_unique]
            contact_gt_binned[name] = centers[gt_unique]

        # --- plot estimated contact dots ---
        plt.figure(figsize=(10,6))
        y_pos = {name: idx for idx, name in enumerate(self.leg_names)}
        for name in self.leg_names:
            times = contact_binned[name]
            plt.scatter(times,
                        [y_pos[name]] * len(times),
                        marker='o',
                        label=name)
        plt.yticks(list(y_pos.values()), list(y_pos.keys()))
        plt.xlabel('Time since start (s)')
        plt.ylabel('Leg')
        plt.title('Estimated Foot Contact (0.5 s bins)')
        plt.ylim(-0.5, len(self.leg_names)-0.5)
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.02,1), loc='upper left')

        # --- plot ground-truth contact dots ---
        plt.figure(figsize=(10,6))
        for name in self.leg_names:
            times_gt = contact_gt_binned[name]
            plt.scatter(times_gt,
                        [y_pos[name]] * len(times_gt),
                        marker='o',
                        label=name)
        plt.yticks(list(y_pos.values()), list(y_pos.keys()))
        plt.xlabel('Time since start (s)')
        plt.ylabel('Leg')
        plt.title('Ground-Truth Foot Contact (0.5 s bins)')
        plt.ylim(-0.5, len(self.leg_names)-0.5)
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.02,1), loc='upper left')

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
