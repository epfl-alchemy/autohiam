import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import copy
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped


class CartesianMoveNode(Node):
    def __init__(self):
        super().__init__('cartesian_move_node')

        self.declare_parameter('z_offset', 0.05)
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        # Your planning group and end-effector link
        self.group_name = 'arm'
        self.ee_link = 'link6'  # Replace with your actual end-effector link

        # Set up service and action clients
        self.get_scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Wait for all interfaces
        self.get_logger().info('Waiting for services and action servers...')
        self.get_scene_cli.wait_for_service()
        self.fk_client.wait_for_service()
        self.cartesian_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('All clients connected. Proceeding to motion...')

        # Execute the main logic
        self.get_current_pose_and_move()

    def get_current_pose_and_move(self):
        # === Step 1: Get current robot state from planning scene ===
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = scene_req.components.ROBOT_STATE
        future_scene = self.get_scene_cli.call_async(scene_req)
        rclpy.spin_until_future_complete(self, future_scene)
        if not future_scene.result():
            self.get_logger().error('Failed to get planning scene.')
            return
        current_state: RobotState = future_scene.result().scene.robot_state

        # === Step 2: Use FK to get current pose of the end-effector ===
        fk_req = GetPositionFK.Request()
        fk_req.fk_link_names = [self.ee_link]
        fk_req.robot_state = current_state
        fk_req.header.frame_id = 'world'
        future_fk = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future_fk)
        if not future_fk.result():
            self.get_logger().error('FK service call failed.')
            return
        current_pose_stamped: PoseStamped = future_fk.result().pose_stamped[0]
        current_pose = current_pose_stamped.pose

        self.get_logger().info(f"Current EE pose Z: {current_pose.position.z:.4f}")

        # === Step 3: Build Cartesian path request ===
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z = target_pose.position.z + self.z_offset

        cart_req = GetCartesianPath.Request()
        cart_req.group_name = self.group_name
        cart_req.link_name = self.ee_link
        cart_req.header.frame_id = current_pose_stamped.header.frame_id
        cart_req.start_state = current_state
        cart_req.max_step = 0.001  # 1 mm resolution
        cart_req.jump_threshold = 0.0
        cart_req.waypoints.append(target_pose)

        self.get_logger().info(f"Target EE pose Z: {target_pose.position.z:.4f}")

        future_cart = self.cartesian_client.call_async(cart_req)
        rclpy.spin_until_future_complete(self, future_cart)
        if not future_cart.result():
            self.get_logger().error('Cartesian path planning failed.')
            return

        trajectory = future_cart.result().solution

        # === Apply speed scaling ===
        speed_scale = 0.5  # 1.0 = normal speed, 0.5 = slower, 2.0 = faster
        for point in trajectory.joint_trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            t_scaled = t / speed_scale
            point.time_from_start.sec = int(t_scaled)
            point.time_from_start.nanosec = int((t_scaled % 1.0) * 1e9)

        if len(trajectory.joint_trajectory.points) == 0:
            self.get_logger().warn('Cartesian path resulted in 0 trajectory points.')
            return

        self.get_logger().info(f"Path planned with {len(trajectory.joint_trajectory.points)} points.")

        # === Step 4: Execute the trajectory ===
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        exec_future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, exec_future)

        goal_handle = exec_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution goal was rejected.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('Trajectory execution finished.')
        self.get_logger().info('Shutting down after successful execution.')
        self.destroy_node()




def main():
    rclpy.init()
    node = CartesianMoveNode()
    rclpy.shutdown()



if __name__ == '__main__':
    main()