import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import copy
from moveit_msgs.srv import GetPlanningScene, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from builtin_interfaces.msg import Time as TimeMsg


class CartesianMoveNode(Node):
    def __init__(self):
        super().__init__('cartesian_move_node')

        self.declare_parameter('z_offset', 0.05)
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        # Robot-specific configuration
        self.group_name = 'arm'
        self.ee_link = 'link6'
        self.base_link = 'base_link'  # Change to your actual base frame if different

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up service and action clients
        self.get_scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Wait for all interfaces
        self.get_logger().info('Waiting for services and action servers...')
        self.get_scene_cli.wait_for_service()
        self.cartesian_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('All clients connected. Proceeding to motion...')

        # Main logic
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

        # === Step 2: Get EE pose using TF lookup ===
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.ee_link,
                rclpy.time.Time(seconds=0),  # latest available
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {str(e)}")
            return

        current_pose = Pose()
        current_pose.position.x = transform.transform.translation.x
        current_pose.position.y = transform.transform.translation.y
        current_pose.position.z = transform.transform.translation.z
        current_pose.orientation = transform.transform.rotation

        self.get_logger().info(f"Current EE pose Z: {current_pose.position.z:.4f}")

        # === Step 3: Build Cartesian path request ===
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z += self.z_offset

        cart_req = GetCartesianPath.Request()
        cart_req.group_name = self.group_name
        cart_req.link_name = self.ee_link
        cart_req.header.frame_id = self.base_link
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
        speed_scale = 0.5
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
