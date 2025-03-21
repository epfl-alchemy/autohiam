from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    TrajectoryConstraints,
    JointConstraint
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

import threading
#from utils import quaternion_to_euler, euler_to_quaternion
import numpy as np
import quaternion

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    # Compute elements of the rotation matrix
    R = np.array([
        [1 - 2 * y**2 - 2 * z**2,     2 * x * y - 2 * z * w,     2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w,       1 - 2 * x**2 - 2 * z**2,   2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w,       2 * y * z + 2 * x * w,     1 - 2 * x**2 - 2 * y**2]
    ])
    return R

def calculate_new_point(quaternion, point, distance):
    # Convert quaternion to rotation matrix
    R = quaternion_to_rotation_matrix(quaternion)
    # Normal vector in world coordinates (z-axis of frame F)
    normal_vector = R[:, 2]
    # Calculate new coordinates by moving a distance along the normal vector
    new_point = point + distance * normal_vector
    return new_point


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.action_server = "/lbr/move_action"
        self.move_group_name = "arm"
        self.base = "link_0"
        self.end_effector = "link_ee"

        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )

        self.get_logger().info(f"Waiting for action server {self.action_server}...")
        if not self.move_group_action_client.wait_for_server(timeout_sec=5):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )
        self.get_logger().info(f"Done.")

        # marker position
        self.subscription = self.create_subscription(
            Pose,
            'aruco_marker_pose',
            self.listener_callback,
            10)        
        # Robot current state
        self.create_subscription(JointState, 'lbr/joint_states', self.joint_state_callback, 10)

        self.last_command_time = self.get_clock().now()
        self.command_interval = Duration(seconds=5.0)

        self.state_lock = threading.Lock()
    
    def joint_state_callback(self, msg: JointState):
        with self.state_lock:
            self.current_joint_state = msg

    def create_joint_constraints(self) -> List[JointConstraint]:
        joint_constraints = [
            JointConstraint(
                joint_name="A1",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
            JointConstraint(
                joint_name="A2",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
            JointConstraint(
                joint_name="A3",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
            JointConstraint(
                joint_name="A4",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
            JointConstraint(
                joint_name="A5",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
            JointConstraint(
                joint_name="A6",
                position=0.0,  # Target position in radians
                tolerance_above=1.7,  # Tolerance above the target position
                tolerance_below=1.7,  # Tolerance below the target position
                weight=1.0  # Relative importance of this joint constraint
            ),
        ]
        return joint_constraints

    def send_goal_async(self, target: Pose):
        with self.state_lock:
            if not self.current_joint_state or not self.current_joint_state.position:
                self.get_logger().warn("Current joint state is invalid. Aborting the planning request.")
                return
            joint_constraints = self.create_joint_constraints()
            goal = MoveGroup.Goal()
            goal.request.start_state.joint_state = self.current_joint_state  # Set the start state
            goal.request.allowed_planning_time = 5.0
            goal.request.goal_constraints.append(
                Constraints(
                    position_constraints=[
                        PositionConstraint(
                            header=Header(frame_id=self.base),
                            link_name=self.end_effector,
                            constraint_region=BoundingVolume(
                                primitives=[SolidPrimitive(type=2, dimensions=[0.01])],
                                primitive_poses=[Pose(position=target.position)],
                            ),
                            weight=1.0,
                        )
                    ],
                    orientation_constraints=[
                        OrientationConstraint(
                            header=Header(frame_id=self.base),
                            link_name=self.end_effector,
                            orientation=target.orientation,
                            absolute_x_axis_tolerance=0.01,
                            absolute_y_axis_tolerance=0.01,
                            absolute_z_axis_tolerance=0.01,
                            weight=1.0,
                        )
                    ],
                    #joint_constraints=joint_constraints
                )
            )
            goal.request.group_name = self.move_group_name
            goal.request.max_acceleration_scaling_factor = 0.1
            goal.request.max_velocity_scaling_factor = 0.5
            goal.request.num_planning_attempts = 200

            return self.move_group_action_client.send_goal_async(goal)
    

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        if current_time - self.last_command_time > self.command_interval:
            self.get_logger().info('Received marker pose. Position: {}, Orientation: {}'.format(
                (msg.position.x, msg.position.y, msg.position.z),
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            ))
            initial_quat = np.quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            rotate_x_180 = np.quaternion(0, 0, 1, 0)
            result_quat0 = rotate_x_180 * initial_quat
            result_quat = quaternion.as_float_array(result_quat0)

            quat_values = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) 
            point = np.array([msg.position.x, msg.position.y, msg.position.z])
            distance = 0.5
            new_coordinates = calculate_new_point(quat_values, point, distance)
            x_new = new_coordinates[0]
            y_new = new_coordinates[1]
            z_new = new_coordinates[2]

            target_pose = Pose(
                #position=Point(x=round(msg.position.x,2), y=round(msg.position.y,2), z=round(msg.position.z,2) + 0.50),
                position=Point(x=round(x_new,2), y=round(y_new,2), z=round(z_new,2)),
                orientation=Quaternion(x=result_quat[0], y=result_quat[1], z=result_quat[2], w=result_quat[3])
            )

            # Send the goal asynchronously to the Move Group action server
            future_goal_handle = self.send_goal_async(target_pose)
            future_goal_handle.add_done_callback(self.goal_response_callback)  # Handle the response

            # Update the last command time
            self.last_command_time = current_time
        else:
            self.get_logger().info("Waiting to send next goal. Time since last command: {:.2f} seconds".format(
                (current_time - self.last_command_time).nanoseconds / 1e9))
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        else:
            self.get_logger().info('Goal accepted :)')

def main(args: List = None) -> None:
    rclpy.init(args=args)
    move_group_action_client_node = MoveGroupActionClientNode("move_group_action_client_node")
    
    try:
        rclpy.spin(move_group_action_client_node)  # Handles callbacks and waits for events
    except KeyboardInterrupt:
        move_group_action_client_node.get_logger().info('Node stopped cleanly')
    except BaseException as e:
        move_group_action_client_node.get_logger().error('Exception: ' + str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()