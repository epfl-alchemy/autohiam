import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


import yaml

class cuRoboExecNode(Node):

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

    def execute(self, positions: list, velocities: list, accelerations: list, dt):
        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        for i in range(6):
            joint_trajectory_goal.trajectory.joint_names.append(f"joint{i + 1}")

        time_from_start_sec = 0

        for i, position in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = position

            if velocities:
                point.velocities = velocities[i]
            if accelerations:
                point.accelerations = accelerations[i]
            
            point.time_from_start.sec = int(time_from_start_sec)
            point.time_from_start.nanosec = int((time_from_start_sec - int(time_from_start_sec)) * 1e9)
            
            joint_trajectory_goal.trajectory.points.append(point)
            time_from_start_sec += dt

        # send goal
        goal_future = self.joint_trajectory_action_client_.send_goal_async(
            joint_trajectory_goal,
            #feedback_callback=self.feedback_callback
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
            self, result_future, timeout_sec=(len(positions) - 1) * dt + goal_sec_tolerance
        )

        if result_future.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error("Failed to execute joint trajectory.")
            return

    def feedback_callback(self, feedback_msg):
        # Log the currently executing joint positions from the feedback
        feedback = feedback_msg.feedback
        current_positions = feedback.actual.positions
        self.get_logger().info(f"Executing joint positions: {current_positions}")

    def get_motion(self, filename):
        with open(filename, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
        return data
    

def main(args: list = None) -> None:
    rclpy.init(args=args)
    curobo_exec_node = cuRoboExecNode(
        "joint_trajectory_executioner_node"
    )

    positions = '/home/szhuang/autohiam_ws/positions_list.yaml'
    velocities = '/home/szhuang/autohiam_ws/velocities_list.yaml'
    accelerations = '/home/szhuang/autohiam_ws/accelerations_list.yaml'
    dt_path = '/home/szhuang/autohiam_ws/optimized_dt.yaml'

    positions = curobo_exec_node.get_motion(positions)
    velocities = curobo_exec_node.get_motion(velocities)
    accelerations = curobo_exec_node.get_motion(accelerations)
    dt = curobo_exec_node.get_motion(dt_path)
    print(len(positions))

    curobo_exec_node.execute(positions,velocities,accelerations,dt)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()