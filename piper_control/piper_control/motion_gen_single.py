import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

#cuRobo
import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as cuPose
from curobo.types.robot import JointState as cuJointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from piper_control.utils import quaternion_to_rotation_matrix, calculate_new_point

import numpy as np
import quaternion
import yaml
from scipy.spatial.transform import Rotation as R

class cuRoboGenNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.tensor_args = TensorDeviceType()
        self.declare_parameter('robot', 'piper_no_gripper.yml')
        self.declare_parameter('time_dilation_factor', 1.0)
        self.declare_parameter('interpolation_dt', 0.005)
        self.declare_parameter('collision_cache_mesh', 20)
        self.declare_parameter('collision_cache_cuboid', 20)
        self.declare_parameter('pose_change_threshold', 0.01)
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.curobo_warmup()
        self.js_buffer = None

    def joint_state_callback(self, msg):
        self.js_buffer = {
            'position': msg.position,
            'velocity': msg.velocity,
        }
        
    def curobo_warmup(self):
        robot_file = self.get_parameter('robot').get_parameter_value().string_value
        if robot_file == '':
            self.get_logger().fatal('Received empty robot file')
            raise SystemExit        
        collision_cache_cuboid = (
            self.get_parameter('collision_cache_cuboid').get_parameter_value().integer_value
        )
        collision_cache_mesh = (
            self.get_parameter('collision_cache_mesh').get_parameter_value().integer_value
        )
        interpolation_dt = (
            self.get_parameter('interpolation_dt').get_parameter_value().double_value
        )
        self.get_logger().info('warming up cuRobo, wait until ready')
        tensor_args = self.tensor_args

        world_file = WorldConfig.from_dict(
            {
                'cuboid': {
                    'table': {
                        'pose': [0, 0, -0.05, 1, 0, 0, 0],  # x, y, z, qw, qx, qy, qz
                        'dims': [4.0, 4.0, 0.1],
                    }
                },
            }
        )
        
        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_file,
            world_file,
            tensor_args,
            interpolation_dt=interpolation_dt,
            collision_cache={
                'obb': collision_cache_cuboid,
                'mesh': collision_cache_mesh,
            },
            #collision_checker_type=CollisionCheckerType.VOXEL,
            optimize_dt = False
        )

        motion_gen = MotionGen(motion_gen_cfg)
        motion_gen.warmup(enable_graph=True)
        self.motion_gen = motion_gen
        self.get_logger().info('cuRobo is ready for planning queries!')

    def update_world_objects(self):
        #world_update_status = True
        self.get_logger().info("update_world_objects")
        cuboid_list = [
            #Cuboid(name="obs_1", pose=[0.5, 0, 0.275, 1, 0, 0, 0], dims=[0.20, 0.27, 0.55]),
            #Cuboid(name="obs_2", pose=[0.5, 0, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
            #Cuboid(name="obs_3", pose=[0.3, 0.3, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
            #Cuboid(name="obs_4", pose=[0, 0.5, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5])
        ]
        sphere_list = []
        cylinder_list = []
        mesh_list = []
        
        self.world_model = WorldConfig(
            cuboid=cuboid_list,
            cylinder=cylinder_list,
            sphere=sphere_list,
            mesh=mesh_list,
        ).get_collision_check_world()
        self.motion_gen.update_world(self.world_model)
        
    def curobo_motion(self, js):
        if self.js_buffer is None:
            self.get_logger().info("Waiting for current joint state to be available.")
            return
        self.get_logger().info('Generating and optimizing trajectory...')
        time_dilation_factor = (
            self.get_parameter('time_dilation_factor').get_parameter_value().double_value
        )
        self.update_world_objects()
        state = cuJointState.from_position(
            self.tensor_args.to_device(torch.tensor(self.js_buffer['position'], dtype=torch.float32, device='cuda:0')).unsqueeze(0),
            joint_names=[
                "joint2",
                "joint3",
                "joint5",
                "joint6",
                "joint1",
                "joint4",
            ],
        )
        state.velocity = self.tensor_args.to_device(torch.tensor(self.js_buffer['velocity'], dtype=torch.float32, device='cuda:0')).unsqueeze(0)

        start_state = self.motion_gen.get_active_js(state)
        
        goal_pose = cuPose(
            position = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32, device='cuda:0'),
            quaternion = torch.tensor([[1.0, 0.0, 0.0, 0.0]], dtype=torch.float32, device='cuda:0') #wxyz
        )

        goal_state = cuJointState.from_position(
            self.tensor_args.to_device(torch.tensor([[0.1579, 0.8164, -0.89, 0.0194, 0.5611, -0.0194]], dtype=torch.float32, device='cuda:0')),
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ],
        )

        if js:
            motion_gen_result = self.motion_gen.plan_single_js(
                start_state,
                goal_state,
                MotionGenPlanConfig(
                    max_attempts=10, 
                    enable_graph_attempt=5, 
                    time_dilation_factor=time_dilation_factor),
            )
        else:
            motion_gen_result = self.motion_gen.plan_single(
                start_state,
                goal_pose,
                MotionGenPlanConfig(
                    max_attempts=10,
                    enable_graph_attempt=5,
                    time_dilation_factor=time_dilation_factor,
                ),
            )        
        
        if motion_gen_result.success.item():
            interpolated_solution = motion_gen_result.optimized_plan
            # interpolated_solution = motion_gen_result.interpolated_plan

            joint_positions = interpolated_solution.position.cpu().numpy().tolist()
            joint_velocities = interpolated_solution.velocity.cpu().numpy().tolist()
            joint_accelerations = interpolated_solution.acceleration.cpu().numpy().tolist()
            
            optimized_dt = motion_gen_result.optimized_dt.item()
            # optimized_dt = motion_gen_result.interpolation_dt

            with open('positions_list.yaml', 'w') as yaml_file:
                yaml.dump(joint_positions, yaml_file)
            with open('velocities_list.yaml', 'w') as yaml_file:
                yaml.dump(joint_velocities, yaml_file)
            with open('accelerations_list.yaml', 'w') as yaml_file:
                yaml.dump(joint_accelerations, yaml_file)
            with open('optimized_dt.yaml','w') as yaml_file:
                yaml.dump(optimized_dt, yaml_file) 

        elif not motion_gen_result.valid_query:
            self.get_logger().error(
                f'Invalid planning query: {motion_gen_result.status}'
            )
        else:
            self.get_logger().error(
                f'Motion planning failed wih status: {motion_gen_result.status}'
            ) 


def main(args: list = None) -> None:
    rclpy.init(args=args)
    curobo_gen_node = cuRoboGenNode("curobo_gen_node")

    try:
        # Wait until joint state is received
        while rclpy.ok() and curobo_gen_node.js_buffer is None:
            rclpy.spin_once(curobo_gen_node, timeout_sec=0.1)
            curobo_gen_node.get_logger().info("Waiting for joint states...")

        # Generate trajectory once
        curobo_gen_node.curobo_motion(js=True)

        # Ensure node is properly destroyed before shutdown
        curobo_gen_node.get_logger().info("Trajectory generated, shutting down.")

    except KeyboardInterrupt:
        curobo_gen_node.get_logger().info('KeyboardInterrupt, shutting down.')

    finally:
        curobo_gen_node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == "__main__":
    main()
