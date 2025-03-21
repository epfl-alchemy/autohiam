import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

import time
import math

class PIPERJointTrajectoryExecutionerNode(Node):
    def __init__(
        self,
        node_name: str,
    ) -> None:
        super().__init__(node_name=node_name)
        self.joint_trajectory_action_client_ = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/arm_controller/follow_joint_trajectory",
        )
        while not self.joint_trajectory_action_client_.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def execute(self, positions: list, sec_from_start: int = 15):
        if len(positions) != 6:
            self.get_logger().error("Invalid number of joint positions.")
            return

        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = sec_from_start

        for i in range(6):
            joint_trajectory_goal.trajectory.joint_names.append(f"joint{i + 1}")

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
            self.get_logger().error("Failed to execute joint trajectory.")
            return


def main(args: list = None) -> None:
    rclpy.init(args=args)
    joint_trajectory_executioner_node = PIPERJointTrajectoryExecutionerNode(
        "joint_trajectory_executioner_node"
    )

    poses_degrees = [
        [   0.,     0.,     0.,     0.,   0.,    0.],
    ]

    # Convert each pose from degrees to radians
    poses = [
        [math.radians(angle) for angle in pose] for pose in poses_degrees
    ]

    home_pose = [0.0, 0.0, -0.71, 0.0, 0.7, 0.0]


    # Execute each pose with a 5 second pause in between
    # for pose in poses:
    #     joint_trajectory_executioner_node.get_logger().info(f"Moving to pose: {pose}")
    #     joint_trajectory_executioner_node.execute(pose)
    joint_trajectory_executioner_node.execute(home_pose)

    #joint_trajectory_executioner_node.get_logger().info("Moving to zero position.")
    #joint_trajectory_executioner_node.execute(
    #    [
    #        1.8328,  0.9536,  1.3796,  1.2744, -2.2544,  1.5934,  0.2077
    #    ]
    #)
    
    #joint_trajectory_executioner_node.get_logger().info("Moving to useful position.")
    #joint_trajectory_executioner_node.execute(
    #	[
    #	    0.0,
    #	    1.0297,
    #	    -1.4137,
    #	    0.9425,
    #	    2.4609,
    #	    0.1396,
    #	    -0.6807
    #	]
    #)

    rclpy.shutdown()