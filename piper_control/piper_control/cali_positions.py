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


    poses = [
        [-0.7, 0.966, -1.533, 0.362, 0.987, -0.375],  # middle 0, 0.753, -0.859, 0.0, 0.131, -0.028
        [0.0, 0.972, -1.584, 0.016, 1.001, 0.009], # middle:left 0.517, 0.753, -0.859, -0.937, 0.241, 0.939
        [0.258, 0.966, -1.533, -0.565, 1.109, 0.504], # middle:right -0.595, 0.753, -0.859, 0.912, 0.131, -0.892
        [-0.595, 0.753, -0.859, 0.912, 0.131, -0.892],  # from up looking down: middle 0.0, 0.972, -1.584, 0.016, 1.001, 0.009
        [0.0, 0.753, -0.859, 0.0, 0.131, -0.028],  # from up looking down: left -0.7, 0.966, -1.533, 0.362, 0.987, -0.375
        [0.517, 0.753, -0.859, -0.937, 0.241, 0.939],  # from up looking down: right 0.258, 0.966, -1.533, -0.565, 1.109, 0.504
        [-0.446, 0.232, -0.320, -0.493, -0.443, -0.047], # from down looking up 0., 0., 0., 0., -0.547, 0.0
        [0.0, 0.0, 0.0, 0.0, -0.547, 0.0],   # from down looking up: left  -0.446, 0.232, -0.320, -0.493, -0.443, -0.047
        [0.446, 0.148, -0.091, 0.518, -0.684,0.038],   # from down looking up: right  0.446, 0.148, -0.091, 0.518, -0.684,0.038
    ]

    # simulated marker position 0.5309, 0, 0.2592
    # poses = [
    #     [0.1579, 0.8164, -0.89, 0.0194, 0.5611, -0.0194],
    #     [0.0143, 0.8164, -0.89, 0.0194, 0.5611, -0.0194],
    #     [-0.1750, 0.8164, -0.89, 0.0194, 0.5611, -0.0194],
    #     [-0.2250, 0.5338, -0.4450, 0.0194, 0.0976, -0.0194],
    #     [0.0143, 0.5338, -0.4450, 0.0194, 0.0976, -0.0194],
    #     [0.2057, 0.5338, -0.4450, 0.0194, 0.0976, -0.0194],
    #     [0.2536, 0.1256, -0.0890, 0.0194, -0.2196, -0.0194],
    #     [0.0143, 0.1256, -0.0890, 0.0194, -0.2196, -0.0194],
    #     [-0.2250, 0.1256, -0.0890, 0.0194, -0.2196, -0.0194]
    # ]

    # Convert each pose from degrees to radians
    # poses = [
    #     [math.radians(angle) for angle in pose] for pose in poses_degrees
    # ]


    # Execute each pose with a 5 second pause in between
    for pose in poses:
        joint_trajectory_executioner_node.get_logger().info(f"Moving to pose: {pose}")
        joint_trajectory_executioner_node.execute(pose)
        time.sleep(5)  # Pause for 5 seconds


    #joint_trajectory_executioner_node.get_logger().info("Moving to useful position.")
    #joint_trajectory_executioner_node.execute(
    #    [
    #        0.0,
    #        0.0,
    #        0.0,
    #        -1.57,
    #        0.0,
    #        1.57,
    #        0.0,
    #    ]
    #)

    rclpy.shutdown()