import os
import shutil
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import jax
from flax.training.train_state import TrainState
import orbax.checkpoint
import optax

from unitree_locomotion.policy import get_policy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from hb40_commons.msg import BridgeData
from hb40_commons.msg import JointCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType


class UnitreeLocomotion(Node):
    def __init__(self):
        super().__init__("unitree_locomotion")

        self.is_real_robot = False
        self.is_tuda_robot = True

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Which robot we are using
        self.declare_parameter("robot", "a1")
        self.robot = self.get_parameter("robot").get_parameter_value().string_value
        self.get_logger().info(f'Locomotion node started for {self.robot}')

        self.bridge_data_subscription = self.create_subscription(
            BridgeData,
            f"/{self.robot}/bridge_data",
            self.bridge_data_callback,
            qos_profile=qos_profile)
        
        self.velocity_command_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.velocity_command_callback,
            qos_profile=qos_profile)
        
        self.joint_commands_publisher = self.create_publisher(
            JointCommand,
            f"/{self.robot}/joint_command" if self.is_real_robot else f"/{self.robot}/joint_commandHighPrio",
            qos_profile=qos_profile)
        
        self.joint_positions = np.zeros(13)
        self.joint_velocities = np.zeros(13)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity = np.zeros(3)
        self.x_goal_velocity = 0.0
        self.y_goal_velocity = 0.0
        self.yaw_goal_velocity = 0.0

        self.kp = [20.0,] * 12
        self.kd = [0.5,] * 12
        self.scaling_factor = 0.25
        
        if self.robot == "a1":
            self.nominal_joint_positions = np.array([-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5, 0.0])
        elif self.robot == "go1":
            self.nominal_joint_positions = np.array([-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5, 0.0])
        elif self.robot == "go2":
            self.nominal_joint_positions = np.array([-0.1, -0.8, 1.5, 0.1, 0.8, -1.5, 0.1, -1.0, 1.5, -0.1, 1.0, -1.5, 0.0])
            
        self.max_joint_velocities = np.array([
            25.0, 25.0, 25.0,
            25.0, 25.0, 25.0,
            25.0, 25.0, 25.0,
            25.0, 25.0, 25.0,
            3.5
        ])

        # self.mask_from_real_to_obssim = [3, 4, 5, 0, 1, 2, 6, 7, 8, 9, 10, 11]
        # self.mask_from_xmlsim_to_real = [7, 8, 9, 10, 11, 1, 2, 3, 4, 5, 6, 0]
        # self.mask_from_xmlsim_to_obssim = [10, 11, 7, 8, 9, 1, 2, 3, 4, 5, 6, 0]
        # self.mask_from_real_to_obssim = [3, 4, 5, 0, 1, 2, 6, 7, 8, 9, 10, 11, 12]
        # self.mask_from_xmlsim_to_real = [7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6, 0]
        # self.mask_from_xmlsim_to_obssim = [10, 11, 12, 7, 8, 9, 1, 2, 3, 4, 5, 6, 0] 
        
        self.mask_from_real_to_obssim = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 12]
        self.mask_from_xmlsim_to_real = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        self.mask_from_xmlsim_to_obssim = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 12]
        
        if self.robot == "go2":
            self.mask_from_real_to_obssim = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 12]
            self.mask_from_xmlsim_to_real = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 12]
            self.mask_from_xmlsim_to_obssim = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]



        self.previous_action = np.zeros(13)

        # jax.config.update("jax_platform_name", "cpu")

        # neural_network_path = "plane.model"
        package_share_directory = get_package_share_directory('unitree_locomotion')

        self.action_space = 12
        if self.robot == "a1":
            neural_network_path = os.path.join(package_share_directory, 'resource', "E25__A1__plane__default_domain_rand__v1.model")
        elif self.robot == "go1":
            neural_network_path = os.path.join(package_share_directory, 'resource', "E25__GO1__plane__default_domain_rand__v1.model")
        elif self.robot == "go2":
            neural_network_path = os.path.join(package_share_directory, 'resource', "E25__GO2__plane__default_domain_rand__v1.model")

        splitted_path = neural_network_path.split("/")
        checkpoint_dir = "/".join(splitted_path[:-1]) if len(splitted_path) > 1 else "."
        checkpoint_file_name = splitted_path[-1]

        shutil.unpack_archive(f"{checkpoint_dir}/{checkpoint_file_name}", f"{checkpoint_dir}/tmp", "zip")
        checkpoint_dir = f"{checkpoint_dir}/tmp"
        jax_model_file_name = [f for f in os.listdir(checkpoint_dir) if "checkpoint" in f][0]

        check_point_handler = orbax.checkpoint.PyTreeCheckpointHandler(aggregate_filename=jax_model_file_name)
        checkpointer = orbax.checkpoint.Checkpointer(check_point_handler)

        # loaded_algorithm_config = checkpointer.restore(checkpoint_dir)["config_algorithm"]
        with open(f"{checkpoint_dir}/config_algorithm.json", "r") as f:
            loaded_algorithm_config = json.load(f)
            

        self.policy = get_policy(loaded_algorithm_config, self.action_space)

        self.policy.apply = jax.jit(self.policy.apply)

        key = jax.random.PRNGKey(0)
        key, policy_key = jax.random.split(key, 2)

        dummy_state = np.zeros((1, 48))

        self.policy_state = TrainState.create(
            apply_fn=self.policy.apply,
            params=self.policy.init(policy_key, dummy_state),
            tx=optax.chain(
                optax.clip_by_global_norm(0.0),
                optax.inject_hyperparams(optax.adam)(learning_rate=lambda count: 0.0),
            )
        )

        # call to jit compile
        self.policy.apply(self.policy_state.params, dummy_state)

        target = {
            "policy": self.policy_state,
        }
        checkpoint = checkpointer.restore(checkpoint_dir, item=target)
        self.policy_state = checkpoint["policy"]

        shutil.rmtree(checkpoint_dir)

        self.nn_active = True

        print(f"Robot ready. Using device: {jax.default_backend()}")

        # timer_period = 0.01  # 100 Hz
        timer_period = 0.02  # 50 Hz
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(timer_period, self.timer_callback, clock=self.system_clock)


    def bridge_data_callback(self, msg):
        self.joint_positions = np.array(msg.joint_position)
        self.joint_velocities = np.array(msg.joint_velocity)
        if self.is_real_robot and self.is_tuda_robot:
            self.joint_positions[-1] *= -1.0
            self.joint_velocities[-1] *= -1.0
        self.orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])


    def velocity_command_callback(self, msg):
        self.x_goal_velocity = msg.linear.x
        self.y_goal_velocity = msg.linear.y
        self.yaw_goal_velocity = msg.angular.z * 0.5  # Topic gives -2 to 2, but we want -1 to 1


    def timer_callback(self):
        if not self.nn_active:
            return
        
        
        if self.x_goal_velocity != 0.0 or self.y_goal_velocity != 0.0 or self.yaw_goal_velocity != 0.0:
            use_policy = True
        else:
            use_policy = False

        joint_command_msg = JointCommand()
        if not use_policy:
            joint_command_msg.header.stamp = self.get_clock().now().to_msg()
            # joint_command_msg.kp = self.kp
            # joint_command_msg.kd = self.kd
            joint_command_msg.kp = [50.0,] * 12
            joint_command_msg.kd = [5.0,] * 12
            joint_command_msg.t_pos = self.nominal_joint_positions[:-1].tolist()
            joint_command_msg.t_vel = [0.0,] * 12
            joint_command_msg.t_trq = [0.0,] * 12
        
        else:
            qpos = (self.joint_positions - self.nominal_joint_positions) / 3.14
            qpos = qpos[self.mask_from_real_to_obssim]
            qvel = self.joint_velocities / self.max_joint_velocities
            qvel = qvel[self.mask_from_real_to_obssim]
            previous_action = self.previous_action[self.mask_from_xmlsim_to_obssim] / 3.14
            qpos_qvel_previous_action = np.vstack((qpos, qvel, previous_action)).T.flatten()

            ang_vel = self.angular_velocity / 10.0
            orientation_quat_inv = R.from_quat(self.orientation).inv()
            projected_gravity_vector = orientation_quat_inv.apply(np.array([0.0, 0.0, -1.0]))

            observation = np.concatenate([
                qpos_qvel_previous_action, ang_vel,
                [self.x_goal_velocity, self.y_goal_velocity, self.yaw_goal_velocity],
                projected_gravity_vector
            ])
            
            action = jax.device_get(self.policy.apply(self.policy_state.params, observation))
            action = np.append(action, 0.0)
            robot_action = action[self.mask_from_xmlsim_to_real]

            target_joint_positions = self.nominal_joint_positions + self.scaling_factor * robot_action
            if self.is_real_robot and self.is_tuda_robot:
                target_joint_positions[-1] *= -1.0

            joint_command_msg.header.stamp = self.get_clock().now().to_msg()
            # joint_command_msg.source_node = "nn_controller"
            joint_command_msg.kp = self.kp
            joint_command_msg.kd = self.kd
            joint_command_msg.t_pos = target_joint_positions[:-1].tolist()
            joint_command_msg.t_vel = [0.0,] * 12
            joint_command_msg.t_trq = [0.0,] * 12

            self.previous_action = action

        self.joint_commands_publisher.publish(joint_command_msg)



def main(args=None):
    rclpy.init(args=args)
    robot_handler = UnitreeLocomotion()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()