import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from turtlebot4_custom_msg.msg import TrialInfo  # import custom message

# import re
# import os
# import yaml

class TrialStates:
    
    # Trial states
    s_TRIAL_IDLE = 0
    s_TRIAL_RUNNING = 1

    # Command actions
    a_TRIAL_RESET = 0
    a_TRIAL_RUN = 1
    a_TRIAL_PAUSE = 2
    a_ROBOTS_RESET = 3
    a_ROBOTS_FLOCK = 4
    a_ROBOTS_FREEZE = 5

#TODO finish: controller allows trials to be started/stopped more easily
# additionally, allows for simultaneous control of robot behaviour
class TrialController(Node):


    def __init__(self):
        super().__init__('control_node')

        # Listen for triggers
        trigger_sub = self.create_subscription(
            TrialInfo,
            '/trial_info',
            self.create_trial_callback,
            10
        )


def main(args=None):
    rclpy.init(args=args)
    control_node = TrialController()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
