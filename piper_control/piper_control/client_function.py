import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from curobo_ros2.srv import GenerateTrajectory
import time

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        
        # Create the service client with an appropriate timeout
        self.cli = self.create_client(GenerateTrajectory, 'generate_trajectory')
        
        # Wait for the service to become available
        self.get_logger().info('Waiting for trajectory generation service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service is available, sending request')
        self.send_request()
        
    def send_request(self):
        # Create service request
        request = GenerateTrajectory.Request()
        
        # Set goal pose
        request.goal_pose = Pose()
        request.goal_pose.position.x = 0.061297
        request.goal_pose.position.y = -0.069672
        request.goal_pose.position.z = 0.339075
        request.goal_pose.orientation.w = 0.807163
        request.goal_pose.orientation.x = -0.001325
        request.goal_pose.orientation.y = 0.590326
        request.goal_pose.orientation.z = 0.000750
        
        # Set to use pose-based planning rather than joint-based
        request.use_joint_state = False
        
        # Send request asynchronously
        self.get_logger().info('Sending trajectory generation request...')
        future = self.cli.call_async(request)
        
        # Define a timeout for waiting for the response
        timeout_sec = 20.0  # Adjust based on how long your planning might take
        
        # Wait for the response with timeout
        start_time = self.get_clock().now()
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if future is done
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
                
                # Exit the loop once we have a response
                break
            
            # Check for timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(f'Service call timed out after {timeout_sec} seconds')
                break
            
            # Small sleep to prevent CPU from being hogged
            time.sleep(0.01)
        
        self.get_logger().info('Client node completed operation and will now shut down')

def main():
    rclpy.init()
    
    # Create and use the client
    client = TrajectoryClient()
    
    # Clean up
    client.destroy_node()
    rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()