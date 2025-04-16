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
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo_ros2.srv import GenerateTrajectory

import itertools
from array import array

import threading
import time
import traceback


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

        self.joint_trajectory_action_client = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/arm_controller/follow_joint_trajectory",
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
        try:
            if self.js_buffer is None:
                self.get_logger().warn("Current joint state not yet received.")
                response.success = False
                response.status = "Joint state not yet received"
                return response

            time_dilation_factor = (
                self.get_parameter('time_dilation_factor').get_parameter_value().double_value
            )

            # Update world
            self.get_logger().info("=== Step 1: updated world ===")
            self.update_world_objects()

            # Convert Pose to cuPose
            self.get_logger().info("=== Step 2: building goal pose ===")
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
            self.get_logger().info("=== Step 3: planning ===")
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
                self.get_logger().info("=== Step 4: extracting plan ===")
                plan_data = plan.optimized_plan
                # response.trajectory_positions = plan_data.position.cpu().numpy().tolist()
                # response.trajectory_velocities = plan_data.velocity.cpu().numpy().tolist()
                # response.trajectory_accelerations = plan_data.acceleration.cpu().numpy().tolist()
                positions = plan_data.position.cpu().numpy().tolist()
                velocities = plan_data.velocity.cpu().numpy().tolist()
                accelerations = plan_data.acceleration.cpu().numpy().tolist()
                dt = plan.optimized_dt.item()
                

                response.trajectory_positions = list(itertools.chain.from_iterable(positions))
                response.trajectory_velocities = list(itertools.chain.from_iterable(velocities))
                response.trajectory_accelerations = list(itertools.chain.from_iterable(accelerations))
                response.dt = dt

                response.success = True
                response.status = "Success"

                # exec_ok, exec_status = self.execute_trajectory(
                # response.trajectory_positions,
                # response.trajectory_velocities,
                # response.trajectory_accelerations,
                # response.dt
                # )

                def reshape(flat_array, width=6):
                    return [array('d', flat_array[i:i+width]) for i in range(0, len(flat_array), width)]

                self.get_logger().info("=== Step 5: reshaping ===")
                positions = reshape(response.trajectory_positions)
                velocities = reshape(response.trajectory_velocities)
                accelerations = reshape(response.trajectory_accelerations)

                self.get_logger().info("=== Step 6: executing trajectory ===")
                exec_ok, exec_status = self.execute_trajectory(
                    positions, velocities, accelerations, response.dt
                )

                # trajectory_msg = JointTrajectory()
                # trajectory_msg.joint_names = [f"joint{i + 1}" for i in range(6)]

                # time_from_start = 0.0
                # for i in range(len(trajectory_positions)):
                #     point = JointTrajectoryPoint()
                #     point.positions = trajectory_positions[i]
                #     point.velocities = trajectory_velocities[i]
                #     point.accelerations = trajectory_accelerations[i]
                #     point.time_from_start.sec = int(time_from_start)
                #     point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
                #     trajectory_msg.points.append(point)
                #     time_from_start += response.dt

                # exec_ok, exec_status = self.execute_trajectory(trajectory_msg)


                if not exec_ok:
                    response.success = False
                    response.status = f"Planning OK, but {exec_status}"
                else:
                    self.get_logger().info("=== Step 7: Done ===")
                    response.success = True
                    response.status = "Planning and execution successful"
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
        
    def execute_trajectory(self, positions, velocities, accelerations, dt):
        try:
            self.get_logger().info("Creating trajectory goal...")
            
            goal = FollowJointTrajectory.Goal()
            goal_sec_tolerance = 1
            goal.goal_time_tolerance.sec = goal_sec_tolerance

            # Add joint names
            for i in range(6):
                goal.trajectory.joint_names.append(f"joint{i + 1}")

            # Add trajectory points
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

            # Send goal and wait for result
            future = self.joint_trajectory_action_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            try:
                goal_handle = future.result(timeout=20.0)
                if not goal_handle.accepted:
                    self.get_logger().error("Goal was rejected by server")
                    return False, "Execution goal rejected"
                    
                result_future = goal_handle.get_result_async()
                try:
                    result = result_future.result(timeout=30.0)
                    if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                        return True, "Execution succeeded"
                    else:
                        return False, f"Execution failed (code {result.result.error_code})"
                except TimeoutError:
                    return False, "Timeout waiting for execution result"
                    
            except TimeoutError:
                return False, "Timeout waiting for goal acceptance"
                
        except Exception as e:
            self.get_logger().error(f"Exception in execute_trajectory: {e}")
            return False, f"Exception: {str(e)}"
        
    # def execute_trajectory(self, positions, velocities, accelerations, dt):
    #     try:
    #         self.get_logger().info("Creating trajectory goal...")
            
    #         goal = FollowJointTrajectory.Goal()
    #         goal_sec_tolerance = 1
    #         goal.goal_time_tolerance.sec = goal_sec_tolerance

    #         # Add joint names
    #         self.get_logger().info("Adding joint names...")
    #         for i in range(6):
    #             goal.trajectory.joint_names.append(f"joint{i + 1}")

    #         # Add trajectory points
    #         self.get_logger().info(f"Adding {len(positions)} trajectory points...")
    #         time_from_start_sec = 0.0
    #         for i, position in enumerate(positions):
    #             # Only add logging for first and last points to avoid flooding
    #             if i == 0 or i == len(positions)-1:
    #                 self.get_logger().info(f"Point {i}: pos={position[:3]}...")
                    
    #             point = JointTrajectoryPoint()
    #             point.positions = position
    #             if velocities:
    #                 point.velocities = velocities[i]
    #             if accelerations:
    #                 point.accelerations = accelerations[i]
    #             point.time_from_start.sec = int(time_from_start_sec)
    #             point.time_from_start.nanosec = int((time_from_start_sec - int(time_from_start_sec)) * 1e9)
    #             goal.trajectory.points.append(point)
    #             time_from_start_sec += dt

    #         # Send goal
    #         self.get_logger().info("About to send goal...")
    #         try:
    #             future = self.joint_trajectory_action_client.send_goal_async(goal)
    #             self.get_logger().info("Goal sent, waiting for acceptance...")
    #         except Exception as e:
    #             self.get_logger().error(f"Exception during send_goal_async: {e}")
    #             return False, f"Failed to send goal: {str(e)}"

    #         # Wait for acceptance with timeout
    #         try:
    #             # Use a shorter timeout for goal acceptance
    #             acceptance_timeout = 20.0
    #             self.get_logger().info(f"Spinning with {acceptance_timeout}s timeout for goal acceptance...")
    #             spin_result = rclpy.spin_until_future_complete(self, future, timeout_sec=acceptance_timeout)
    #             self.get_logger().info(f"Spin for goal acceptance completed with result: {spin_result}")

    #         except Exception as e:
    #             self.get_logger().error(f"Exception during spin for goal acceptance: {e}")
    #             return False, f"Exception during goal acceptance: {str(e)}"

    #         if not future.done():
    #             self.get_logger().error("Timed out waiting for goal acceptance")
    #             return False, "Goal acceptance timed out"

    #         try:
    #             goal_handle = future.result()
    #             if goal_handle is None:
    #                 self.get_logger().error("Received None goal handle")
    #                 return False, "Received None goal handle"
                    
    #             self.get_logger().info(f"Goal handle accepted: {goal_handle.accepted}")
                
    #             if not goal_handle.accepted:
    #                 self.get_logger().error("Goal was rejected by server")
    #                 return False, "Execution goal rejected"
                    
    #             self.get_logger().info("Goal was accepted by server, requesting result...")
                
    #         except Exception as e:
    #             self.get_logger().error(f"Exception processing goal handle: {e}")
    #             return False, f"Exception processing goal handle: {str(e)}"

    #         # Rest of the method unchanged...
    #         # (For brevity - this is where you'd handle the result_future)
            
    #         # Placeholder for successful execution
    #         self.get_logger().info("Trajectory execution succeeded.")
    #         return True, "Execution succeeded"
            
    #     except Exception as e:
    #         self.get_logger().error(f"Top-level exception in execute_trajectory: {e}")
    #         return False, f"Exception: {str(e)}"

    # def execute_trajectory(self, trajectory):
    #     goal = FollowJointTrajectory.Goal()
    #     goal.trajectory = trajectory

    #     future = self.joint_trajectory_action_client.send_goal_async(goal)
    #     rclpy.spin_until_future_complete(self, future)
    #     goal_handle = future.result()

    #     if not goal_handle.accepted:
    #         self.get_logger().error("Trajectory execution goal was rejected.")
    #         return False, "Execution goal rejected"

    #     result_future = goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, result_future)
    #     result = result_future.result().result

    #     if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
    #         self.get_logger().info("Trajectory execution succeeded.")
    #         return True, "Execution succeeded"
    #     else:
    #         self.get_logger().error(f"Execution failed with error code: {result.error_code}")
    #         return False, f"Execution failed (code {result.error_code})"



def main():
    rclpy.init()

    curobo_gen_service = cuRoboGenService("curobo_gen_service")

    # rclpy.spin(curobo_gen_service)

    executor = MultiThreadedExecutor()
    executor.add_node(curobo_gen_service)
    try:
        curobo_gen_service.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        curobo_gen_service.get_logger().info('Keyboard interrupt, shutting down.\n')

    rclpy.shutdown()


if __name__ == '__main__':
    main()