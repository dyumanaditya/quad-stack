import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
import numpy as np
from scipy.interpolate import interp1d

class OdometryPlotter(Node):
    def __init__(self):
        super().__init__('odometry_plotter')
        
        # Subscriber for /odom_kinematics
        self.odom_kinematics_subscriber = self.create_subscription(
            Odometry,
            '/odom_kinematics',
            self.odom_kinematics_callback,
            10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Subscriber for /odom_gt (ground truth)
        self.odom_gt_subscriber = self.create_subscription(
            Odometry,
            '/odom_gt',
            self.odom_gt_callback,
            10)
        
        self.base_lin_vel_subscriber = self.create_subscription(
            TwistStamped,
            '/base_lin_vel',
            self.base_lin_vel_callback,
            10)
        
        # Subscriber for /joint_states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Data for plotting and saving
        self.kinematics_data = []
        self.odom_data = []
        self.gt_data = []
        self.kinematics_x = []
        self.kinematics_y = []
        self.kinematics_z = []
        self.odom_x = []
        self.odom_y = []
        self.odom_z = []
        self.gt_x = []
        self.gt_y = []
        self.gt_z = []
        # Variables to store velocity data
        self.vel_x = []
        self.vel_y = []
        self.vel_z = []
        # Data for joint states
        self.joint_velocities = []
        self.joint_names = []

    def odom_kinematics_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.kinematics_data.append((timestamp, x, y, z))
        self.kinematics_x.append(x)
        self.kinematics_y.append(y)
        self.kinematics_z.append(z)
    
    def odom_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.odom_data.append((timestamp, x, y, z))
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.odom_z.append(z)

    def odom_gt_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.gt_data.append((timestamp, x, y, z))
        self.gt_x.append(x)
        self.gt_y.append(y)
        self.gt_z.append(z)
        
    def base_lin_vel_callback(self, msg):
        # Extract linear velocities
        self.vel_x.append(msg.twist.linear.x)
        self.vel_y.append(msg.twist.linear.y)
        self.vel_z.append(msg.twist.linear.z)
        
    def joint_state_callback(self, msg):
        if not self.joint_names:
            self.joint_names = list(msg.name)
        self.joint_velocities.append(list(msg.velocity))

    def synchronize_data(self, odom, gt):
        # Convert lists to numpy arrays for interpolation
        kinematics_data = np.array(odom)
        gt_data = np.array(gt)
        
        # Interpolate ground truth data to match kinematics timestamps
        if len(gt_data) == 0 or len(kinematics_data) == 0:
            self.get_logger().warn("No data received to compute error.")
            return None, None
        
        gt_interp_x = interp1d(gt_data[:, 0], gt_data[:, 1], fill_value="extrapolate")
        gt_interp_y = interp1d(gt_data[:, 0], gt_data[:, 2], fill_value="extrapolate")
        gt_interp_z = interp1d(gt_data[:, 0], gt_data[:, 3], fill_value="extrapolate")

        gt_x_synced = gt_interp_x(kinematics_data[:, 0])
        gt_y_synced = gt_interp_y(kinematics_data[:, 0])
        gt_z_synced = gt_interp_z(kinematics_data[:, 0])

        return kinematics_data[:, 1:], np.vstack((gt_x_synced, gt_y_synced, gt_z_synced)).T

    def save_data(self, filename='odometry_data.pkl'):
        data = {
            'kinematics_data': self.kinematics_data,
            'odom_data': self.odom_data,
            'gt_data': self.gt_data,
            'velocity_data': {
                'vel_x': self.vel_x,
                'vel_y': self.vel_y,
                'vel_z': self.vel_z
            }
        }
        with open(filename, 'wb') as f:
            pickle.dump(data, f)
        self.get_logger().info(f'Data saved to {filename}')

    def plot_path(self):
        kinematics_positions, gt_positions = self.synchronize_data(self.kinematics_data, self.gt_data)

        if kinematics_positions is None or gt_positions is None:
            return

        plt.figure(figsize=(10, 5))

        # Subplot 1: Odometry Path vs Ground Truth
        plt.subplot(1, 2, 1)
        plt.plot(self.kinematics_x, self.kinematics_y, label="Odometry Kinematics")
        plt.plot(self.gt_x, self.gt_y, label="Ground Truth")
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Odometry Path vs Ground Truth')
        plt.legend()
        plt.grid(True)

        # Subplot 2: Error between Odometry and Ground Truth
        # print(kinematics_positions[0])
        # print()
        # print(gt_positions[0])
        plt.subplot(1, 2, 2)
        error = np.sqrt((kinematics_positions[4:, 0] - gt_positions[4:, 0])**2 +
                        (kinematics_positions[4:, 1] - gt_positions[4:, 1])**2 +
                        (kinematics_positions[4:, 2] - gt_positions[4:, 2])**2)
        plt.plot(error, label="Position Error")
        plt.xlabel('Sample')
        plt.ylabel('Error (m)')
        plt.title('Odometry vs Ground Truth Error')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('/home/ws/results/kinematics-odom.png')
        plt.show()

    def plot_odom_path(self):
        odom_positions, gt_positions = self.synchronize_data(self.odom_data, self.gt_data)

        if odom_positions is None or gt_positions is None:
            return

        plt.figure(figsize=(10, 5))

        # Subplot 1: Odometry Path vs Ground Truth
        plt.subplot(1, 2, 1)
        plt.plot(self.odom_x, self.odom_y, label="Odometry")
        plt.plot(self.gt_x, self.gt_y, label="Ground Truth")
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Odometry Path vs Ground Truth')
        plt.legend()
        plt.grid(True)

        # Subplot 2: Error between Odometry and Ground Truth
        plt.subplot(1, 2, 2)
        error = np.sqrt((odom_positions[:, 0] - gt_positions[:, 0])**2 +
                        (odom_positions[:, 1] - gt_positions[:, 1])**2 +
                        (odom_positions[:, 2] - gt_positions[:, 2])**2)
        plt.plot(error, label="Position Error")
        plt.xlabel('Sample')
        plt.ylabel('Error (m)')
        plt.title('Odometry vs Ground Truth Error')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('/home/ws/results/vo-odom.png')
        plt.show()

    def plot_3d_path_kinematics(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plotting the paths in 3D
        ax.plot(self.kinematics_x, self.kinematics_y, self.kinematics_z, label="Odometry Kinematics")
        ax.plot(self.gt_x, self.gt_y, self.gt_z, label="Ground Truth")

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title('3D Odometry Path vs Ground Truth')
        ax.set_xlim(-4.0, -1.0)
        ax.set_ylim(1.0, 4.0)
        ax.set_zlim(0.0, 1.6)
        ax.legend()
        plt.savefig('/home/ws/results/3d-kinematics-odom.png')
        plt.show()
    
    def plot_3d_path_odom(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plotting the paths in 3D
        ax.plot(self.odom_x, self.odom_y, self.odom_z, label="Visual Odometry")
        ax.plot(self.gt_x, self.gt_y, self.gt_z, label="Ground Truth")

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title('3D Odometry Path vs Ground Truth')
        ax.legend()
        plt.savefig('/home/ws/results/3d-vo-odom.png')
        plt.show()
        
    def plot_velocities(self):
        # Plot linear velocities in X, Y, and Z directions
        plt.figure(figsize=(10, 5))
        plt.ylim(-0.1, 0.1)
        
        # Plot X velocity
        plt.subplot(3, 1, 1)
        plt.plot(self.vel_x, label='X Velocity')
        plt.xlabel('Sample')
        plt.ylabel('Velocity (m/s)')
        plt.title('Linear Velocity in X')
        plt.legend()
        plt.grid(True)
        
        # Plot Y velocity
        plt.subplot(3, 1, 2)
        plt.plot(self.vel_y, label='Y Velocity')
        plt.xlabel('Sample')
        plt.ylabel('Velocity (m/s)')
        plt.title('Linear Velocity in Y')
        plt.legend()
        plt.grid(True)
        
        # Plot Z velocity
        plt.subplot(3, 1, 3)
        plt.plot(self.vel_z, label='Z Velocity')
        plt.xlabel('Sample')
        plt.ylabel('Velocity (m/s)')
        plt.title('Linear Velocity in Z')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('/home/ws/results/linear_velocities.png')
        plt.show()
        
    def plot_joint_velocities(self):
        # Check if joint velocity data is available
        if not self.joint_velocities:
            self.get_logger().warn("No joint velocity data received.")
            return

        # Number of joints
        num_joints = len(self.joint_names)

        # Plot joint velocities separately for each joint
        for i in range(num_joints):
            plt.figure(figsize=(8, 4))
            plt.ylim(-0.8, 0.8)
            plt.plot([vel[i] for vel in self.joint_velocities], label=f'{self.joint_names[i]} Velocity')
            plt.xlabel('Sample')
            plt.ylabel('Velocity (rad/s)')
            plt.title(f'Joint Velocity: {self.joint_names[i]}')
            plt.legend()
            plt.grid(True)

            # Save the individual plot
            plt.savefig(f'/home/ws/results/joint_velocity_{self.joint_names[i]}.png')
            plt.show()



def main(args=None):
    rclpy.init(args=args)
    odometry_plotter = OdometryPlotter()

    try:
        rclpy.spin(odometry_plotter)
    except KeyboardInterrupt:
        pass

    # Save data before plotting
    odometry_plotter.save_data()

    # Plot paths and errors
    odometry_plotter.plot_path()

    odometry_plotter.plot_odom_path()

    # Plot 3D path
    odometry_plotter.plot_3d_path_kinematics()
    odometry_plotter.plot_3d_path_odom()
    
    # Plot velocity data
    odometry_plotter.plot_velocities()
    
    # Plot joint state data
    odometry_plotter.plot_joint_velocities()

    odometry_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()