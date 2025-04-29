import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

import time
import math

class GripperExecutionerNode(Node):
    def __init__(
        self,
        node_name: str,
    ) -> None:
        super().__init__(node_name=node_name)
        self.joint_trajectory_action_client_ = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/gripper_controller/follow_joint_trajectory",
        )
        while not self.joint_trajectory_action_client_.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def execute(self, positions: list, sec_from_start: int = 5):
        if len(positions) != 1:
            self.get_logger().error("Invalid number of gripper joint positions.")
            return

        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = sec_from_start

        # Specify only gripper joints: joint7 and joint8
        joint_trajectory_goal.trajectory.joint_names = ["joint7"]

        joint_trajectory_goal.trajectory.points.append(point)

        # send goal
        goal_future = self.joint_trajectory_action_client_.send_goal_async(
            joint_trajectory_goal
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=sec_from_start + goal_sec_tolerance
        )

        if (
            result_future.result().result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error("Failed to execute gripper joint trajectory.")
            return


def main(args: list = None) -> None:
    rclpy.init(args=args)
    gripper_executioner_node = GripperExecutionerNode(
        "gripper_executioner_node"
    )

    # Define gripper positions (in degrees) for joint7 and joint8
    gripper_positions_degrees = [
        [0.0], 
        # [0.04], 
        # [0.0], 
    ]

    # Convert positions from degrees to radians
    # gripper_positions = [
        # [math.radians(angle) for angle in pos] for pos in gripper_positions_degrees
    # ]

    # Execute each gripper position with a 5-second delay between movessbb
    for pos in gripper_positions_degrees:
        gripper_executioner_node.get_logger().info(f"Moving gripper to position: {pos}")
        gripper_executioner_node.execute(pos)

    rclpy.shutdown()
