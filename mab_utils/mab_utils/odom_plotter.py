import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
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
            'gt_data': self.gt_data
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
        plt.subplot(1, 2, 2)
        error = np.sqrt((kinematics_positions[:, 0] - gt_positions[:, 0])**2 +
                        (kinematics_positions[:, 1] - gt_positions[:, 1])**2 +
                        (kinematics_positions[:, 2] - gt_positions[:, 2])**2)
        plt.plot(error, label="Position Error")
        plt.xlabel('Sample')
        plt.ylabel('Error (m)')
        plt.title('Odometry vs Ground Truth Error')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('/home/aditya/results/new/kinematics-odom.png')
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
        plt.savefig('/home/aditya/results/new/vo-odom.png')
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
        ax.legend()
        plt.savefig('/home/aditya/results/new/3d-kinematics-odom.png')
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
        plt.savefig('/home/aditya/results/new/3d-vo-odom.png')
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

    odometry_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
