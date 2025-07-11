import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from curobo_ros2.srv import GenerateTrajectory
from curobo_ros2.msg import MarkerPoseWithID
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import quaternion

from piper_control.utils import quaternion_to_rotation_matrix, calculate_new_point

class MoveIntoHeater(Node):
    def __init__(self):
        super().__init__('move_into_heater')
        self.declare_parameter('marker_id', 34)
        self.expected_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.get_logger().info(f'Looking for marker ID: {self.expected_id}')
        
        # Create the service client with an appropriate timeout
        self.cli = self.create_client(GenerateTrajectory, 'generate_trajectory')
        
        # Wait for the service to become available
        self.get_logger().info('Waiting for curobo trajectory generation and execution service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service is available, sending request')

        self.create_subscription(
            MarkerPoseWithID,
            '/marker_pose_with_id',
            self.marker_listener_callback,
            10)

        self.executed = False  # to ensure we only run once
        self.shutdown_requested = False  # Flag to track shutdown status


    def marker_listener_callback(self, msg: MarkerPoseWithID):
        if self.executed or self.shutdown_requested:
           return

        if self.expected_id is None:
            self.get_logger().info("All steps completed.")
            return

        if msg.id != self.expected_id:
            self.get_logger().info(f"Waiting for marker ID {self.expected_id}, but got {msg.id}.")
            return

        self.get_logger().info(f"Marker {msg.id} detected.")

        self.get_logger().info("Going to pick up the holder...")
        self.executed = True

        x, y, z, qw, qx, qy, qz = self.compute_target_pose(
                msg.marker_pose,
                distance=0.083,
                x_offset=0.0,
                y_offset=0.0,
                z_offset=0.204,
        )
        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.position.z = z
        goal_pose.orientation.w = qw
        goal_pose.orientation.x = qx
        goal_pose.orientation.y = qy
        goal_pose.orientation.z = qz
        # goal_pose.orientation.w = 0.7071
        # goal_pose.orientation.x = 0.0
        # goal_pose.orientation.y = 0.7071
        # goal_pose.orientation.z = 0.0

        # Send the goal to trajectory planner
        self.send_trajectory_request(goal_pose)

    def compute_target_pose(
            self,
            msg: Pose,
            distance,
            x_offset,
            y_offset,
            z_offset,
        ):
        # Process orientation (rotate 180° around X)
        initial_quat = np.quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        rotate_x_180 = np.quaternion(0, 0, 1, 0)
        result_quat0 = rotate_x_180 * initial_quat
        result_quat = quaternion.as_float_array(result_quat0)  #x,y,z,w

        # Compute new position with direction offset
        quat_values = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        point = np.array([msg.position.x, msg.position.y, msg.position.z])

        new_coordinates = calculate_new_point(quat_values, point, distance)
        x_new, y_new, z_new = new_coordinates

        # Apply additional manual offsets
        x = round(x_new + x_offset, 4)
        y = round(y_new + y_offset, 4)
        z = round(z_new + z_offset, 4)

        qw = round(result_quat[3], 4)
        qx = round(result_quat[0], 4)
        qy = round(result_quat[1], 4)
        qz = round(result_quat[2], 4)

        return x, y, z, qw, qx, qy, qz


    def send_trajectory_request(self, pose: Pose):
        request = GenerateTrajectory.Request()
        request.goal_pose = pose
        request.use_joint_state = False

        self.get_logger().info("Sending trajectory request...")
        future = self.cli.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info(f"Step success: {result.status}")
                    self.executed = True
                    self.get_logger().info("Scheduling shutdown...")
                    # Schedule shutdown after 0.5s
                    self.create_timer(0.5, self.request_shutdown)
                else:
                    self.get_logger().error(f"Step failed: {result.status}")
                    self.executed = False
            except Exception as e:
                self.get_logger().error(f"Service call failed: {str(e)}")
                self.executed = False

        future.add_done_callback(callback)

    def request_shutdown(self):
        """Request shutdown but don't actually do it - let main() handle it"""
        self.get_logger().info("Requesting node shutdown...")
        self.shutdown_requested = True
        # Signal the executor to stop
        rclpy.shutdown()

def main():
    rclpy.init()
    node = MoveIntoHeater()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pass

if __name__ == '__main__':
    main()