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

#ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


from curobo_ros2.srv import GenerateTrajectory


class cuRoboGenService(Node):
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

        self.srv = self.create_service(GenerateTrajectory, 'generate_trajectory', self.generate_trajectory_callback)

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

    def generate_trajectory_callback(self, request, response):
        if self.js_buffer is None:
            self.get_logger().warn("Current joint state not yet received.")
            response.success = False
            response.status = "Joint state not yet received"
            return response

        time_dilation_factor = (
            self.get_parameter('time_dilation_factor').get_parameter_value().double_value
        )

        # Update world
        self.update_world_objects()

        # Convert Pose to cuPose
        goal_pose = cuPose(
            position=torch.tensor([[request.goal_pose.position.x,
                                    request.goal_pose.position.y,
                                    request.goal_pose.position.z]], dtype=torch.float32, device='cuda:0'),
            quaternion=torch.tensor([[request.goal_pose.orientation.w,
                                      request.goal_pose.orientation.x,
                                      request.goal_pose.orientation.y,
                                      request.goal_pose.orientation.z]], dtype=torch.float32, device='cuda:0')
        )

        goal_state = cuJointState.from_position(
            self.tensor_args.to_device(torch.tensor([[request.goal_state]], dtype=torch.float32, device='cuda:0')),
            joint_names=[
                "joint2",
                "joint3",
                "joint5",
                "joint6",
                "joint1",
                "joint4",
            ],
        )

        # Get start state
        state = cuJointState.from_position(
            self.tensor_args.to_device(torch.tensor(self.js_buffer['position'], dtype=torch.float32, device='cuda:0')).unsqueeze(0),
            joint_names=[
                "joint2", 
                "joint3", 
                "joint5", 
                "joint6", 
                "joint1", 
                "joint4"]
        )
        state.velocity = self.tensor_args.to_device(torch.tensor(self.js_buffer['velocity'], dtype=torch.float32, device='cuda:0')).unsqueeze(0)
        start_state = self.motion_gen.get_active_js(state)

        # Plan
        if request.use_joint_state:
            # optionally, let client define goal joint state
            self.get_logger().error("Goal joint state planning is not implemented yet in service.")
            response.success = False
            response.status = "Joint state planning not implemented."
            return response
        else:
            plan = self.motion_gen.plan_single(
                start_state,
                goal_pose,
                MotionGenPlanConfig(max_attempts=10, enable_graph_attempt=5, time_dilation_factor=time_dilation_factor)
            )

        if plan.success.item():
            plan_data = plan.optimized_plan
            response.trajectory_positions = plan_data.position.cpu().numpy().flatten().tolist()
            response.trajectory_velocities = plan_data.velocity.cpu().numpy().flatten().tolist()
            response.trajectory_accelerations = plan_data.acceleration.cpu().numpy().flatten().tolist()
            response.dt = plan.optimized_dt.item()
            response.success = True
            response.status = "Success"
        else:
            response.success = False
            response.status = f"Planning failed: {plan.status}"
        
        return response

def main():
    rclpy.init()

    curobo_gen_service = cuRoboGenService("curobo_gen_service")

    rclpy.spin(curobo_gen_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()