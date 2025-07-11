import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from curobo_ros2.srv import GenerateTrajectory
import time

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        
        self.declare_parameter('x', 0.150)
        self.declare_parameter('y', 0.146)
        self.declare_parameter('z', 0.283)

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.z = self.get_parameter('z').get_parameter_value().double_value
        
        self.cli = self.create_client(GenerateTrajectory, 'generate_trajectory')

        self.get_logger().info('Waiting for trajectory generation service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("curobo service is not available, retrying...")
        
        self.get_logger().info('Service is available, sending request')
        self.send_request()
        
    def send_request(self):
        request = GenerateTrajectory.Request()
        
        request.goal_pose = Pose()
        request.goal_pose.position.x = self.x
        request.goal_pose.position.y = self.y
        request.goal_pose.position.z = self.z
        request.goal_pose.orientation.w = 0.4482
        request.goal_pose.orientation.x = 0.0
        request.goal_pose.orientation.y = 0.8939
        request.goal_pose.orientation.z = 0.0
        
        # Set to use pose-based planning rather than joint-based
        request.use_joint_state = False
        
        # Send request asynchronously
        self.get_logger().info('Sending trajectory generation request...')
        future = self.cli.call_async(request)
        
        timeout_sec = 20.0
        
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        self.get_logger().info('Successfully generated and executed trajectory!')
                        self.get_logger().info(f'Status: {response.status}')
                        self.get_logger().info(f'Number of points: {len(response.trajectory_positions) // 6}')
                        self.get_logger().info(f'Trajectory time: {(len(response.trajectory_positions) // 6) * response.dt:.2f} seconds')
                    else:
                        self.get_logger().error(f'Failed to generate/execute trajectory: {response.status}')
                except Exception as e:
                    self.get_logger().error(f'Service call failed with error: {str(e)}')
                
                break
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(f'Service call timed out after {timeout_sec} seconds')
                break
            
            time.sleep(0.01)
        
        self.get_logger().info('Client node completed operation and will now shut down')

def main():
    rclpy.init()

    client = TrajectoryClient()
    
    client.destroy_node()
    rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()