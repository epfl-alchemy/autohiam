#cuRobo
import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid, Mesh, Capsule, Cylinder, Sphere
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as cuPose
from curobo.types.robot import JointState as cuJointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    PoseCostMetric,
)
from curobo.util.usd_helper import UsdHelper

#ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo_ros2.srv import GenerateTrajectory, UpdateObstacle

import itertools
from array import array
from concurrent.futures import Future
import time
from datetime import datetime
import os

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class cuRoboGenService(Node):
    def __init__(self, node_name: str, executor: MultiThreadedExecutor) -> None:
        super().__init__(node_name=node_name)
        self.tensor_args = TensorDeviceType()
        self.declare_parameter('robot', 'piper_with_gripper.yml') #piper_no_gripper.yml
        self.declare_parameter('time_dilation_factor', 0.6)
        self.declare_parameter('interpolation_dt', 0.005)
        self.declare_parameter('collision_cache_mesh', 20)
        self.declare_parameter('collision_cache_cuboid', 20)
        self.declare_parameter('pose_change_threshold', 0.01)
        self.declare_parameter('usd', False)
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.curobo_warmup()
        self.js_buffer = None

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.executor = executor

        self.srv = self.create_service(
            GenerateTrajectory,
            'generate_trajectory',
            self.generate_trajectory_callback,
            callback_group=self.service_group
        )

        self.obstacle_srv = self.create_service(
            UpdateObstacle,
            "update_obstacles",
            self.update_obstacles_callback,
            callback_group=self.service_group
        )

        self.joint_trajectory_action_client = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/arm_controller/follow_joint_trajectory",
            callback_group=self.action_group
        )

        while not self.joint_trajectory_action_client.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def joint_state_callback(self, msg):
        self.js_buffer = {
            'position': msg.position,
            'velocity': msg.velocity,
        }
        
    def curobo_warmup(self):
        self.robot_file = self.get_parameter('robot').get_parameter_value().string_value
        if self.robot_file == '':
            self.get_logger().fatal('Received empty robot file.')
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
        self.get_logger().info('warming up cuRobo, wait until ready...')
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
            self.robot_file,
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
        self.get_logger().info("Updating world objects.")
        cuboid_list = [
            # Cuboid(name="obs_1", pose=[0.35, 0, 0.15, 1, 0, 0, 0], dims=[0.1, 0.1, 0.3]),
            # Cuboid(name="obs_2", pose=[0.5, 0, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
            # Cuboid(name="obs_3", pose=[0.3, 0.3, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5]),
            # Cuboid(name="obs_4", pose=[0, 0.5, 0.25, 1, 0, 0, 0], dims=[0.1, 0.1, 0.5])
        ]
        sphere_list = []
        cylinder_list = [
            # Cylinder(
            #     name="cylinder_1",
            #     radius=0.01875,
            #     height=0.1,
            #     pose=[0.4625, 0.1125, 0.05, 1, 0, 0, 0]
            #     ),
            # Cylinder(
            #     name="cylinder_2",
            #     radius=0.01875,
            #     height=0.1,
            #     pose=[0.4625, 0.1875, 0.05, 1, 0, 0, 0]
            #     ),
        ]
        mesh_list = []
        
        self.world_model = WorldConfig(
            cuboid=cuboid_list,
            cylinder=cylinder_list,
            sphere=sphere_list,
            mesh=mesh_list,
        ).get_collision_check_world()
        self.motion_gen.update_world(self.world_model)

    def generate_trajectory_callback(self, request, response):
        try:
            if self.js_buffer is None:
                self.get_logger().warn("Current joint state not yet received.")
                response.success = False
                response.status = "Joint state not yet received."
                return response

            time_dilation_factor = (
                self.get_parameter('time_dilation_factor').get_parameter_value().double_value
            )

            # Update world
            self.update_world_objects()

            # Get start state
            self.get_logger().info("Getting current joint states.")
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
            self.get_logger().info("Planning collision-free trajectory.")
            if request.use_joint_state:
                self.get_logger().info("Building goal state.")
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

                # pose_cost_metric = PoseCostMetric(
                #     hold_partial_pose=True,
                #     hold_vec_weight=self.motion_gen.tensor_args.to_device([1, 1, 0, 1, 1, 1]),
                # )

                plan_config = MotionGenPlanConfig(
                        max_attempts=10, 
                        enable_graph_attempt=5, 
                        time_dilation_factor=time_dilation_factor)
                
                # plan_config.pose_cost_metric = pose_cost_metric

                plan = self.motion_gen.plan_single_js(
                    start_state,
                    goal_state,
                    plan_config,
                )

                self.get_logger().error("Goal joint state planning is not implemented yet in service.")
                response.success = False
                response.status = "Joint state planning not implemented."
                return response
            else:
                self.get_logger().info("Building goal pose.")
                goal_pose = cuPose(
                    position=torch.tensor([[request.goal_pose.position.x,
                                            request.goal_pose.position.y,
                                            request.goal_pose.position.z]], dtype=torch.float32, device='cuda:0'),
                    quaternion=torch.tensor([[request.goal_pose.orientation.w,
                                            request.goal_pose.orientation.x,
                                            request.goal_pose.orientation.y,
                                            request.goal_pose.orientation.z]], dtype=torch.float32, device='cuda:0')
                )

                plan = self.motion_gen.plan_single(
                    start_state,
                    goal_pose,
                    MotionGenPlanConfig(max_attempts=60, enable_graph_attempt=5, time_dilation_factor=time_dilation_factor, finetune_attempts=15)
                )

            if plan.success.item():
                self.get_logger().info("Planning is successful ,extracting plan results.")
                plan_data = plan.optimized_plan

                positions = plan_data.position.cpu().numpy().tolist()
                velocities = plan_data.velocity.cpu().numpy().tolist()
                accelerations = plan_data.acceleration.cpu().numpy().tolist()
                dt = plan.optimized_dt.item()
                
                # write usd animation
                usd_value = self.get_parameter('usd').get_parameter_value().bool_value
                if usd_value:
                    interpolated_solution = plan.get_interpolated_plan()
                    if interpolated_solution is None:
                        self.get_logger().error("interpolated_solution is None")
                    else:
                        try:
                            self.get_logger().info(f"interpolated_solution has position: {hasattr(interpolated_solution, 'position')}")
                            self.get_logger().info(f"position shape: {interpolated_solution.position.shape}")
                            self.get_logger().info(f"velocity shape: {interpolated_solution.velocity.shape}")
                            self.get_logger().info(f"acceleration shape: {interpolated_solution.acceleration.shape}")
                        except Exception as e:
                            self.get_logger().error(f"Error inspecting interpolated_solution: {e}")

                        try:
                            save_dir = "/home/szhuang/autohiam_ws/src/hiam_gen2/usd_animation"
                            os.makedirs(save_dir, exist_ok=True)
                            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            filename = f"trajectory_{timestamp}.usd"
                            save_path = os.path.join(save_dir, filename)

                            if interpolated_solution is None:
                                    self.get_logger().error("interpolated_solution is None")
                            else:
                                # Make sure positions are contiguous
                                interpolated_solution.position = interpolated_solution.position.contiguous()

                                UsdHelper.write_trajectory_animation(
                                    self.robot_file,
                                    self.world_model,
                                    start_state,
                                    interpolated_solution,
                                    save_path=save_path,
                                    base_frame="/world",
                                    flatten_usd=True,
                                    visualize_robot_spheres=True,
                                    dt=plan.interpolation_dt,
                                )
                                self.get_logger().info(f"USD trajectory saved to: {save_path}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to write USD: {e}")



                response.trajectory_positions = list(itertools.chain.from_iterable(positions))
                response.trajectory_velocities = list(itertools.chain.from_iterable(velocities))
                response.trajectory_accelerations = list(itertools.chain.from_iterable(accelerations))
                response.dt = dt

                response.success = True
                response.status = "Success"

                def reshape(flat_array, width=6):
                    return [array('d', flat_array[i:i+width]) for i in range(0, len(flat_array), width)]

                positions = reshape(response.trajectory_positions)
                velocities = reshape(response.trajectory_velocities)
                accelerations = reshape(response.trajectory_accelerations)

                self.get_logger().info("Executing trajectory, wait until finished.")
                exec_ok, exec_status = self.execute_trajectory(
                    positions, velocities, accelerations, response.dt
                )

                if not exec_ok:
                    response.success = False
                    response.status = f"Planning OK, but {exec_status}"
                else:
                    self.get_logger().info("Motion planning and execution successful.")
                    response.success = True
                    response.status = "Planning and execution successful!!!"
                    self.get_logger().info(response.status)

            else:
                response.success = False
                response.status = f"Planning failed: {plan.status}"
            
            return response
        except Exception as e:
            self.get_logger().error(f"Exception during trajectory callback: {e}")
            response.success = False
            response.status = f"Exception: {str(e)}"
            return response

    # def update_obstacles_callback(self, request, response):
    #     try:
    #         num_obs = len(request.names)
    #         cuboid_list = []
    #         for i in range(num_obs):
    #             pose = request.poses[i*7:(i+1)*7]
    #             dims = request.dims[i*3:(i+1)*3]
    #             cuboid = Cuboid(name=request.names[i], pose=pose, dims=dims)
    #             cuboid_list.append(cuboid)

    #         self.world_model = WorldConfig(
    #             cuboid=cuboid_list,
    #             cylinder=[], sphere=[], mesh=[],
    #         ).get_collision_check_world()

    #         self.motion_gen.update_world(self.world_model)

    #         response.success = True
    #         response.status = "Obstacle list updated successfully."
    #         self.get_logger().info(response.status)
    #     except Exception as e:
    #         response.success = False
    #         response.status = f"Error: {str(e)}"
    #         self.get_logger().error(response.status)

    #     return response

    def update_obstacles_callback(self, request, response):
        try:
            # Maintain a persistent dictionary of cuboid objects
            if not hasattr(self, 'current_cuboids'):
                self.current_cuboids = {}

            if request.mode == "replace":
                self.current_cuboids.clear()
                for i in range(len(request.names)):
                    pose = request.poses[i*7:(i+1)*7]
                    dims = request.dims[i*3:(i+1)*3]
                    self.current_cuboids[request.names[i]] = Cuboid(name=request.names[i], pose=pose, dims=dims)

            elif request.mode == "add":
                for i in range(len(request.names)):
                    pose = request.poses[i*7:(i+1)*7]
                    dims = request.dims[i*3:(i+1)*3]
                    self.current_cuboids[request.names[i]] = Cuboid(name=request.names[i], pose=pose, dims=dims)

            elif request.mode == "delete":
                for name in request.names:
                    if name in self.current_cuboids:
                        del self.current_cuboids[name]

            else:
                response.success = False
                response.status = f"Invalid mode '{request.mode}', must be 'add', 'replace', or 'delete'"
                self.get_logger().error(response.status)
                return response

            # Rebuild and update world model
            updated_world = WorldConfig(
                cuboid=list(self.current_cuboids.values()),
                cylinder=[], sphere=[], mesh=[]
            )
            self.world_model = updated_world.get_collision_check_world()
            self.motion_gen.update_world(self.world_model)

            response.success = True
            response.status = f"Obstacles updated with mode: {request.mode}"
            self.get_logger().info(response.status)
            self.log_current_obstacles("/home/szhuang/autohiam_ws/src/hiam_gen2/current_world.obj")

        except Exception as e:
            response.success = False
            response.status = f"Exception: {str(e)}"
            self.get_logger().error(response.status)
            self.log_current_obstacles("/home/szhuang/autohiam_ws/src/hiam_gen2/current_world.obj")

        return response

    # def log_current_obstacles(self):
    #     if not hasattr(self, 'current_cuboids') or not self.current_cuboids:
    #         self.get_logger().info("No cuboid obstacles currently in the world.")
    #     else:
    #         self.get_logger().info("Current cuboid obstacles:")
    #         for name, cuboid in self.current_cuboids.items():
    #             self.get_logger().info(f"  - Name: {name}, Pose: {cuboid.pose}, Dims: {cuboid.dims}")
    def log_current_obstacles(self, save_path: str = None):
        if not hasattr(self, 'current_cuboids') or not self.current_cuboids:
            self.get_logger().info("No cuboid obstacles currently in the world.")

            if save_path:
                try:
                    world_config = WorldConfig(
                        cuboid=list(self.current_cuboids.values()),
                        cylinder=[], sphere=[], mesh=[], capsule=[]
                    )
                    world_config.save_world_as_mesh(save_path)
                    self.get_logger().info(f"Saved world mesh to: {save_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save mesh: {e}")
        else:
            self.get_logger().info("Current cuboid obstacles:")
            for name, cuboid in self.current_cuboids.items():
                self.get_logger().info(f"  - Name: {name}, Pose: {cuboid.pose}, Dims: {cuboid.dims}")

            if save_path:
                try:
                    world_config = WorldConfig(
                        cuboid=list(self.current_cuboids.values()),
                        cylinder=[], sphere=[], mesh=[], capsule=[]
                    )
                    world_config.save_world_as_mesh(save_path)
                    self.get_logger().info(f"Saved world mesh to: {save_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save mesh: {e}")

    def execute_trajectory(self, positions, velocities, accelerations, dt):
        try:
            goal = FollowJointTrajectory.Goal()
            goal.goal_time_tolerance.sec = 1
            goal.trajectory.joint_names = [f"joint{i + 1}" for i in range(6)]

            time_from_start_sec = 0.0
            for i, position in enumerate(positions):
                point = JointTrajectoryPoint()
                point.positions = position
                if velocities:
                    point.velocities = velocities[i]
                if accelerations:
                    point.accelerations = accelerations[i]
                point.time_from_start.sec = int(time_from_start_sec)
                point.time_from_start.nanosec = int((time_from_start_sec - int(time_from_start_sec)) * 1e9)
                goal.trajectory.points.append(point)
                time_from_start_sec += dt

            if not self.joint_trajectory_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Action server not ready")
                return False, "Action server not ready"

            self.get_logger().info("Sending trajectory goal...")
            result_future = Future()

            def send_goal_done_callback(fut):
                goal_handle = fut.result()
                if not goal_handle.accepted:
                    self.get_logger().error("Goal was rejected")
                    result_future.set_result((False, "Goal rejected"))
                    return

                self.get_logger().info("Goal accepted. Waiting for result...")

                result_fut = goal_handle.get_result_async()

                def result_done_callback(res_fut):
                    result = res_fut.result()
                    if result is None:
                        self.get_logger().error("Result is None")
                        result_future.set_result((False, "Result is None"))
                        return

                    if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                        self.get_logger().info("Trajectory execution succeeded")
                        result_future.set_result((True, "Execution succeeded"))
                    else:
                        self.get_logger().error(f"Execution failed: {result.result.error_code}")
                        result_future.set_result((False, f"Execution failed (code {result.result.error_code})"))

                result_fut.add_done_callback(result_done_callback)

            send_goal_fut = self.joint_trajectory_action_client.send_goal_async(goal)
            send_goal_fut.add_done_callback(send_goal_done_callback)

            # Wait for result in a non-blocking way
            while not result_future.done():
                time.sleep(0.01)

            return result_future.result()

        except Exception as e:
            self.get_logger().error(f"Exception in execute_trajectory: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return False, f"Exception: {str(e)}"
        
def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    curobo_gen_service = cuRoboGenService("curobo_gen_service", executor)
    executor.add_node(curobo_gen_service)
    curobo_gen_service.get_logger().info('Beginning client, shut down with CTRL-C')

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()