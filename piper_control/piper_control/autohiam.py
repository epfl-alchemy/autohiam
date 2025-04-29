import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from curobo_ros2.srv import GenerateTrajectory
from curobo_ros2.msg import MarkerPoseWithID
from rclpy.executors import MultiThreadedExecutor

import time
import numpy as np
import quaternion

from piper_control.utils import quaternion_to_rotation_matrix, calculate_new_point

class AUTOHIAM(Node):
    def __init__(self):
        super().__init__('autohiam')
        
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
        
        self.current_step = 1
        self.executing = False

        self.step_marker_map = {
            1: 10,  # Step 1 expects marker ID 10
            2: 23, 
            3: 67 
        }


    def marker_listener_callback(self, msg: MarkerPoseWithID):
        if self.executing:
            return  # Skip if still executing a previous step

        expected_id = self.step_marker_map.get(self.current_step, None)
        if expected_id is None:
            self.get_logger().info("All steps completed.")
            return

        if msg.id != expected_id:
            self.get_logger().info(f"Waiting for marker ID {expected_id}, but got {msg.id}.")
            return

        self.get_logger().info(f"Step {self.current_step}: Marker {msg.id} detected.")
        self.executing = True  # Lock execution

        # Build the goal pose based on step
        if self.current_step == 1:
            self.get_logger().info("Step 1")
            goal_pose = Pose()
            goal_pose.position.x = 0.07309
            goal_pose.position.y = 7.37672e-06
            goal_pose.position.z = 0.27727
            goal_pose.orientation.w = 0.5810569
            goal_pose.orientation.x = 2.7419946e-06
            goal_pose.orientation.y = 0.8138629
            goal_pose.orientation.z = -4.924522e-06

        elif self.current_step == 2:
            self.get_logger().info("Step 2")
            x, y, z, qw, qx, qy, qz = self.compute_target_pose(
                msg.marker_pose,
                distance=0.3,
                x_offset=0.0,
                y_offset=0.0,
                z_offset=0.0
            )
            goal_pose = Pose()
            goal_pose.position.x = x
            goal_pose.position.y = y
            goal_pose.position.z = z
            goal_pose.orientation.w = qw
            goal_pose.orientation.x = qx
            goal_pose.orientation.y = qy
            goal_pose.orientation.z = qz

        else:
            self.get_logger().info("No implementation for current step.")
            self.executing = False
            return

        # Send the goal to trajectory planner
        self.send_trajectory_request(goal_pose)

    def compute_target_pose(
            self,
            msg: Pose,
            distance=0.3,
            x_offset=0.0,
            y_offset=0.0,
            z_offset=0.0,
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

        qw = result_quat[3]
        qx = result_quat[0]
        qy = result_quat[1]
        qz = result_quat[2]

        return x, y, z, qw, qx, qy, qz


    def send_trajectory_request(self, pose: Pose):
        request = GenerateTrajectory.Request()
        request.goal_pose = pose
        request.use_joint_state = False

        self.get_logger().info("Sending trajectory request...")  # NEW log
        future = self.cli.call_async(request)
        step_number = self.current_step

        def callback(fut):
            self.get_logger().info(f"Callback triggered for step {step_number}")  # NEW log
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info(f"Step {step_number} success: {result.status}")
                    self.current_step += 1
                else:
                    self.get_logger().error(f"Step {step_number} failed: {result.status}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {str(e)}")
            finally:
                self.executing = False

        future.add_done_callback(callback)


def main():
    rclpy.init()
    node = AUTOHIAM()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()