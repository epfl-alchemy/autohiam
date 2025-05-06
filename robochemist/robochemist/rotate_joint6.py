import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from threading import Event


class RotateJointNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)

        # Joint state subscriber
        self.joint_positions = [0.0] * 6
        self.joint_names = [f"joint{i+1}" for i in range(6)]
        self.joint_state_event = Event()
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Action client
        self.joint_trajectory_action_client_ = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        while not self.joint_trajectory_action_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def joint_state_callback(self, msg: JointState):
        joint_pos_map = dict(zip(msg.name, msg.position))
        try:
            self.joint_positions = [joint_pos_map[name] for name in self.joint_names]
            self.joint_state_event.set()
        except KeyError:
            pass  # Some joints not available yet

    def move_joint6_relative(self, delta: float, sec_from_start: int = 3):
        if not self.joint_state_event.wait(timeout=10):
            self.get_logger().error("Timed out waiting for /joint_states.")
            return

        # Compute new joint positions
        new_positions = self.joint_positions.copy()
        new_positions[5] += delta  # Only update joint 6

        self.get_logger().info(f"Current joint 6: {self.joint_positions[5]:.4f}, moving to {new_positions[5]:.4f}")
        self.execute(new_positions, sec_from_start)

    def execute(self, positions: list, sec_from_start: int = 10):
        if len(positions) != 6:
            self.get_logger().error("Expected 6 joint positions.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance.sec = 1

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = sec_from_start

        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points.append(point)

        # Send goal
        goal_future = self.joint_trajectory_action_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=sec_from_start + 2)

        if result_future.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory executed successfully.")
        else:
            self.get_logger().error("Trajectory execution failed.")


def main(args=None):
    rclpy.init(args=args)
    node = RotateJointNode("joint_trajectory_executioner_node")

    # Rotate joint 6 by +0.1 radians
    node.move_joint6_relative(0.1)

    rclpy.shutdown()
