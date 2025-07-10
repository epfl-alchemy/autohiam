import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose
from curobo_ros2.srv import GenerateTrajectory
from rclpy.executors import MultiThreadedExecutor

class CartesianControlClient(Node):
    def __init__(self):
        super().__init__('cartesian_control_client')
        self.declare_parameter('x_offset', 0.00)
        self.declare_parameter('y_offset', 0.00)
        self.declare_parameter('z_offset', 0.00)
        self.x_offset = self.get_parameter('x_offset').get_parameter_value().double_value
        self.y_offset = self.get_parameter('y_offset').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.base_link = "base_link"
        self.end_effector_link = "link6"

        self.cli = self.create_client(GenerateTrajectory, 'generate_trajectory')
        self.get_logger().info("Waiting for cumotion service...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time())

            current_pos = transform.transform.translation
            current_ori = transform.transform.rotation

            self.get_logger().info(f'Current X: {current_pos.x:.3f}, moving {self.x_offset}m...')
            self.get_logger().info(f'Current Y: {current_pos.y:.3f}, moving {self.y_offset}m...')
            self.get_logger().info(f'Current Z: {current_pos.z:.3f}, moving {self.z_offset}m...')

            # Cancel timer to avoid duplicate calls
            self.timer.cancel()

            # Create a new Pose with Z decreased
            goal_pose = Pose()
            goal_pose.position.x = current_pos.x + self.x_offset
            goal_pose.position.y = current_pos.y + self.y_offset
            goal_pose.position.z = current_pos.z + self.z_offset
            goal_pose.orientation.w = current_ori.w
            goal_pose.orientation.x = current_ori.x
            goal_pose.orientation.y = current_ori.y
            goal_pose.orientation.z = current_ori.z

            request = GenerateTrajectory.Request()
            request.goal_pose = goal_pose
            request.use_joint_state = False

            future = self.cli.call_async(request)

            def callback(fut):
                try:
                    result = fut.result()
                    if result.success:
                        self.get_logger().info(f"Step success: {result.status}")
                        self.get_logger().info("Scheduling shutdown...")
                        # Schedule shutdown after 0.5s
                        self.create_timer(0.5, self.request_shutdown)
                    else:
                        self.get_logger().error(f"Step failed: {result.status}")
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {str(e)}")

            future.add_done_callback(callback)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")

    def request_shutdown(self):
        """Request shutdown but don't actually do it - let main() handle it"""
        self.get_logger().info("Requesting node shutdown...")
        self.shutdown_requested = True
        # Signal the executor to stop
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CartesianControlClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pass
if __name__ == "__main__":
    main()
