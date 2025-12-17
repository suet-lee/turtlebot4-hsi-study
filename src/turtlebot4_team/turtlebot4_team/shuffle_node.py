import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped

import math
import time

class RobotShuffle(Node):
    """
    A ROS2 Node for testing control from a central PC server.

    This node implements a "shuffle" movement which is controlled via topic message.
    """

    STATE_IDLE = 0
    STATE_SHUFFLE_SMALL = 1
    STATE_SHUFFLE_BIG = 2
    STATE_SPIN = 3

    def __init__(self):
        super().__init__('robot_shuffle')

        # Get the robot's namespace from the ROS parameter server
        self.namespace = self.get_namespace()

        if self.namespace == "/":
            self.get_logger().info(f"Missing namespace")
            self.namespace = ""

        # Subscribes to this robot's team info
        self.create_subscription(Int8, f'{self.namespace}/shuffle_state', self.shuffle_callback, 10)
        self.get_logger().info(f"{self.namespace} ready to shuffle")
        
        self.control_publisher = self.create_publisher(TwistStamped, f'{self.namespace}/cmd_vel', 10)
        
        # Movement constraints and interaction parameters
        self.max_forward_velocity = 1.0

        self.state = self.STATE_IDLE

    def shuffle_callback(self, msg):
        new_state = int(msg.data)
        if self.state == new_state:
            return
        
        cycle = math.floor(time.time())%2
        if cycle == 0:
            x_factor = 1.
        if cycle == 1:
            x_factor = -1. # :(

        if new_state == self.STATE_SHUFFLE_SMALL:
            state_msg = "shuffle small"
            x = x_factor
            z = 0.
        elif new_state == self.STATE_SHUFFLE_BIG:
            state_msg = "shuffle big"
            x = x_factor*2.
            z = 0.
        elif new_state == self.STATE_SPIN:
            state_msg = "spin :)"
            x = 0.
            z = 1.
        else:
            state_msg = "take a break"
            x = 0.
            z = 0.

        self.get_logger().info(f"Now {state_msg}")
        
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.angular.z = z
        self.control_publisher.publish(twist)


def main(args=None):
    """
    Main function to initialize and run the RobotShuffle ROS2 node.
    """
    rclpy.init(args=args)
    robot_shuffle_node = RobotShuffle()
    rclpy.spin(robot_shuffle_node)
    robot_shuffle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
