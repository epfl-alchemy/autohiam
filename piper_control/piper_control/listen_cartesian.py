import rclpy
from rclpy.node import Node
import tf2_ros

class EndEffectorPositionSubscriber(Node):
    def __init__(self):
        super().__init__('end_effector_position_subscriber')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_current_pose)
        
        # Getting the names of base_link and end_effector_link as defined in the URDF file
        self.base_link = "base_link"
        self.end_effector_link = "link6"

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,  # Robot's base link
                self.end_effector_link,  # Robot's end effector link
                rclpy.time.Time())

            # Extract position
            current_position = transform.transform.translation
            # Extract orientation (rotation in quaternion)
            current_orientation = transform.transform.rotation
            
            self.get_logger().info(
                "Current End Effector Position: x={}, y={}, z={}".format(
                    current_position.x,
                    current_position.y,
                    current_position.z) + 
                " and Orientation: x={}, y={}, z={}, w={}".format(
                    current_orientation.x,
                    current_orientation.y,
                    current_orientation.z,
                    current_orientation.w))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Failed to lookup transform: %s" % str(e))

def main(args=None):
    rclpy.init(args=args)
    end_effector_position_subscriber = EndEffectorPositionSubscriber()
    rclpy.spin(end_effector_position_subscriber)
    end_effector_position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()