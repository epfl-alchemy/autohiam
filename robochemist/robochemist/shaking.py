import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor

class ShakingNode(Node):
    def __init__(self, node_name="shaking_node"):
        super().__init__(node_name=node_name)
        self.joint_trajectory_action_client_ = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/arm_controller/follow_joint_trajectory",
        )
        
        # Store the latest joint state
        self.current_positions = {}
        self.controlled_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # Subscribe to joint states
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Wait for the server to become available
        while not self.joint_trajectory_action_client_.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

        time.sleep(5)

        # Shaking parameters
        self.amplitude = 0.07  # 7 cm shaking amplitude (in radians)
        self.frequency = 2.5   # 2 Hz shaking
        self.duration = 7.0    # 7 seconds
        self.sample_rate = 40  # 50 Hz for smooth motion
        self.joint_name = "joint6"  # Only target joint
        self.executed = False

    def joint_state_callback(self, msg):
        # Capture current joint positions for controlled joints only
        for name, position in zip(msg.name, msg.position):
            if name in self.controlled_joints:
                self.current_positions[name] = position

        # Start the shaking once the initial positions are known
        if set(self.controlled_joints).issubset(self.current_positions.keys()) and not self.executed:
            self.shake()

    def shake(self):
        start_time = time.time()
        self.get_logger().info(f"Starting shake. Initial joint positions: {self.current_positions}")
        
        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            angle = self.amplitude * np.sin(2 * np.pi * self.frequency * t)

            # Prepare the joint positions
            target_positions = [self.current_positions[j] for j in self.controlled_joints]
            joint_index = self.controlled_joints.index(self.joint_name)

            # This line cause shaking to center around 0 not current joint 6 position
            # target_positions[joint_index] = angle
            initial_joint6 = self.current_positions[self.joint_name]
            target_positions[joint_index] = initial_joint6 + angle

            self.get_logger().info(
                f"Shaking joint {self.joint_name} around {initial_joint6:.3f} with offset {angle:.3f}, result: {initial_joint6 + angle:.3f}"
            )

            # joint_state_log = ", ".join([f"{name}: {pos:.3f}" for name, pos in zip(self.controlled_joints, target_positions)])
            # self.get_logger().info(f"Sending trajectory point: {joint_state_log}")
            
            # Create the trajectory goal
            joint_trajectory_goal = FollowJointTrajectory.Goal()
            joint_trajectory_goal.goal_time_tolerance.sec = 1
            
            # Set joint positions
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * len(self.controlled_joints)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(1.0 / self.sample_rate * 1e9)

            # Add point to the trajectory
            joint_trajectory_goal.trajectory.joint_names = self.controlled_joints
            joint_trajectory_goal.trajectory.points.append(point)

            # Send the goal asynchronously
            future = self.joint_trajectory_action_client_.send_goal_async(joint_trajectory_goal)
            future.add_done_callback(self.goal_response_callback)

            # Log the current angle for debugging
            self.get_logger().info(f"Shaking joint {self.joint_name} at angle: {angle:.3f} radians")

            # Wait briefly before sending the next point
            time.sleep(1.0 / self.sample_rate)

        # Mark as executed to prevent multiple shakes
        self.executed = True

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("Goal accepted by action server.")
                self.create_timer(0.5, self.request_shutdown)
            else:
                self.get_logger().error("Goal rejected by action server.")
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {str(e)}")

    def request_shutdown(self):
        """Request shutdown after delay."""
        self.get_logger().info("Shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ShakingNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

if __name__ == '__main__':
    main()
