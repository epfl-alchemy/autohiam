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
        self.declare_parameter('time_dilation_factor', 0.7)
        self.declare_parameter('interpolation_dt', 0.005)
        self.declare_parameter('collision_cache_mesh', 20)
        self.declare_parameter('collision_cache_cuboid', 20)
        self.declare_parameter('pose_change_threshold', 0.01)
        
        self.create_subscription(
            Pose,
            'aruco_marker_pose',
            self.marker_listener_callback,
            10)        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.target_pose = None
        self.previous_target_pose = None
        
        self.pub = self.create_publisher(Bool, 'motion_generated', 10)
        self.sub = self.create_subscription(Bool, 'execution_done', self.execution_done_callback, 10)
        self.ready_for_new_motion = True
        
        self.curobo_warmup()
        self.__js_buffer = None

    def joint_state_callback(self, msg):
        self.__js_buffer = {
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

    def execution_done_callback(self, msg):
        if msg.data:
            self.ready_for_new_motion = True
            self.get_logger().info("Execution done, ready to generate new motion")
            self.get_logger().info("If the robot doesn't move, it means marker pose change is too small.")

    def marker_listener_callback(self, msg: Pose):
        #self.get_logger().info('Received marker pose. Position: {}, Orientation: {}'.format(
        #    (msg.position.x, msg.position.y, msg.position.z),
        #    (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #))
        initial_quat = np.quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        rotate_x_180 = np.quaternion(0, 0, 1, 0)
        result_quat0 = rotate_x_180 * initial_quat

        result_quat = quaternion.as_float_array(result_quat0) #x,y,z,w

        quat_values = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) 
        point = np.array([msg.position.x, msg.position.y, msg.position.z])

        distance = 0.300
        new_coordinates = calculate_new_point(quat_values, point, distance)
        x_new = new_coordinates[0]
        y_new = new_coordinates[1]
        z_new = new_coordinates[2]
             
        position_tensor = torch.tensor([[round(x_new, 2), round(y_new, 2), round(z_new, 2)]], dtype=torch.float32, device='cuda:0')
        quaternion_tensor = torch.tensor([[result_quat[3], result_quat[0], result_quat[1], result_quat[2]]], dtype=torch.float32, device='cuda:0') #wxyz
        
        new_target_pose = cuPose(
            position=position_tensor,
            quaternion=quaternion_tensor
        )

        self.target_pose = cuPose(
            position=position_tensor,
            quaternion=quaternion_tensor
        )

        threshold = self.get_parameter('pose_change_threshold').get_parameter_value().double_value
        if self.poses_are_similar(self.target_pose, new_target_pose, threshold):
            self.get_logger().info("Target pose change is too small, skipping motion generation.")

        self.previous_target_pose = self.target_pose
        self.target_pose = new_target_pose

        self.curobo_motion()
        
    def curobo_motion(self):
        if self.target_pose is None or self.__js_buffer is None:
            self.get_logger().info("Waiting for both target pose and current joint state to be available.")
            return
        if not self.ready_for_new_motion:
            self.get_logger().info("Waiting for execution to finish before generating new motion.")
            #self.pub.publish(Bool(data=True))
            return
        self.get_logger().info('Generating and optimizing trajectory...')
        time_dilation_factor = (
            self.get_parameter('time_dilation_factor').get_parameter_value().double_value
        )
        self.update_world_objects()
        state = cuJointState.from_position(
            self.tensor_args.to_device(torch.tensor(self.__js_buffer['position'], dtype=torch.float32, device='cuda:0')).unsqueeze(0),
            joint_names=[
                "joint2",
                "joint3",
                "joint5",
                "joint6",
                "joint1",
                "joint4",
            ],
        )
        state.velocity = self.tensor_args.to_device(torch.tensor(self.__js_buffer['velocity'], dtype=torch.float32, device='cuda:0')).unsqueeze(0)

        start_state = self.motion_gen.get_active_js(state)
        

        goal_pose = self.target_pose
        motion_gen_result = self.motion_gen.plan_single(
            start_state, 
            goal_pose,
            MotionGenPlanConfig(max_attempts=10, enable_graph_attempt=2, time_dilation_factor=time_dilation_factor),
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

            self.pub.publish(Bool(data=True))
            self.ready_for_new_motion = False
        elif not motion_gen_result.valid_query:
            self.get_logger().error(
                f'Invalid planning query: {motion_gen_result.status}'
            )
        else:
            self.get_logger().error(
                f'Motion planning failed wih status: {motion_gen_result.status}'
            ) 
    
    def poses_are_similar(self, pose1, pose2, threshold):
        if pose1 is None or pose2 is None:
            return False
        pos_diff = torch.norm(pose1.position - pose2.position) #calculates the Euclidean norm (also known as the L2 norm or the magnitude of the difference vector). This gives a single value representing the distance between the two positions in 3D space.
        quat_diff = torch.norm(pose1.quaternion - pose2.quaternion)
        return pos_diff < threshold and quat_diff < threshold        


def main(args: list = None) -> None:
    rclpy.init(args=args)
    curobo_gen_node = cuRoboGenNode(
        "curobo_gen_node"
    )
    try:
        # rclpy.spin(curobo_gen_node)
        while rclpy.ok():
            rclpy.spin_once(curobo_gen_node, timeout_sec=0.1)
            # curobo_gen_node.curobo_motion()      
    except KeyboardInterrupt:
        curobo_gen_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        curobo_gen_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()