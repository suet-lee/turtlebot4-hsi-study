import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8

import random
import yaml
import os
import re


class RobotTeaming(Node):
    """
    A ROS2 Node for coordinating the robot teams.

    This node implements team assignment and re-assignment which enables robots to be shared 
    between teams. It subscribes to information communicated from human operators (via an 
    interface) to determine team (re-)assignments. It also manages the behavioural parameters 
    (such as velocity, density and orientation) for all robots in a given team.

    """

    TASK_EXPLICIT_SHARE = 1
    TASK_AUTONOMOUS_SHARE = 2

    SIGNAL_DEFAULT = 0
    SIGNAL_REQUEST = 1
    SIGNAL_ACCEPT = 2
    SIGNAL_ACCEPT_DELAY = 3
    SIGNAL_TIMEOUT = 4

    def __init__(self):
        """
        Initialize the RobotTeaming node with default parameters and subscriptions.
        """
        super().__init__('robot_teaming')

        self.declare_parameter('task_id', self.TASK_EXPLICIT_SHARE) 
        self.task_id = self.get_parameter('task_id').value

        self.share_requests = {}
        self.share_responses = {}
        self.signal_sent = {}
        self.team_assignments = {}

        # Publishers
        self.interface_pub = {}
        self.team_info_pub = {}

        # Get leader namespace
        share_teams_path = os.path.join(get_package_share_directory(
            'turtlebot4_team'), 'config', 'share_teams.yaml')
        
        with open(share_teams_path, 'r') as f:
            team_data = yaml.full_load(f)

        robot_list = {}
        for team_, team_list in team_data['teams'].items():
            team_id = int(re.search(r'\d', team_).group())
            for robot in team_list:
                robot_list[robot] = team_id

        self.num_teams = len(team_data['leaders'])

        self.get_logger().info(f"Number of teams: {self.num_teams}")
        if self.task_id == self.TASK_EXPLICIT_SHARE:
            self.get_logger().info(f"Task type: Explicit share")
        elif self.task_id == self.TASK_AUTONOMOUS_SHARE:
            self.get_logger().info(f"Task type: Autonomous share")

        # Subscribe to receive interface messages
        for team, leader_namespace in team_data['leaders'].items():
            team_id = int(re.search(r'\d', team).group())
            self.create_subscription(
                Int8,
                f'/interface{team_id}/signal_info/receive',
                self.create_signal_info_callback(team_id),
                10
            )

            # Publishers to send to user interface
            self.interface_pub[team_id] = self.create_publisher(
                Int8,
                f'/interface{team_id}/signal_info/send',
                10
            )

            # Create team info publisher for each leader
            # leader_namespace = f'/leader{i}'
            self.team_info_pub[leader_namespace] = self.create_publisher(
                Int8,
                f'{leader_namespace}/team', 
                10
            )
            msg = Int8()
            msg.data = team_id
            self.team_info_pub[leader_namespace].publish(msg) # Set default team ID of the leader to its ID

        # Create team info publisher for each follower
        for namespace, team_id in robot_list.items():
            follower_namespace = f'/{namespace}'
            self.team_info_pub[follower_namespace] = self.create_publisher(
                Int8,
                f'{follower_namespace}/team', 
                10
            )

            msg = Int8()
            msg.data = team_id
            self.team_info_pub[follower_namespace].publish(msg)
            self.team_assignments[namespace] = team_id


    def create_signal_info_callback(self, interface_id):
        """
        Generates a callback function for handling signals from a user interface.
        
        Args:
            namespace: The ID of the interface.
        
        Returns:
            A callback function that triggers the correct robot sharing mechanism depending on the signal content.
        """
        def signal_info_callback(msg):
            if msg.data == self.SIGNAL_REQUEST:
                if interface_id not in self.share_requests: # Prevent multiple requests
                    self.share_requests[interface_id] = 1
                    self.get_logger().info(f"[INTERFACE SIGNAL] Request from interface {interface_id}")
            elif msg.data == self.SIGNAL_ACCEPT:
                # if interface_id not in self.share_responses:
                self.share_responses[interface_id] = self.SIGNAL_ACCEPT
                self.get_logger().info(f"[INTERFACE SIGNAL] Response from interface {interface_id}: accepted")
            elif msg.data == self.SIGNAL_ACCEPT_DELAY:
                # if interface_id not in self.share_responses:
                self.share_responses[interface_id] = self.SIGNAL_ACCEPT_DELAY
                self.get_logger().info(f"[INTERFACE SIGNAL] Response from interface {interface_id}: accepted with delay")
            else:
                pass

            self.process_signals()
  
        return signal_info_callback


    def process_signals(self):
        if self.task_id == self.TASK_AUTONOMOUS_SHARE:
            return self.share_autonomous()
            
        # Process request signals
        # Only send one request at a time
        # Don't delete request until accepted or declined
        # If a request is not accepted within a time limit, it is automatically declined
        if len(self.share_requests) > 0:
            request_id = random.choice(self.share_requests.keys())
            teams_available = range(self.num_teams)
            del teams_available[request_id]
            if len(teams_available) > 0:
                send_request_to = random.choice(teams_available)
                self.interface_pub[send_request_to].publish(request_id)
                self.signal_sent[send_request_to] = request_id
                self.get_logger().info(f"[SEND REQUEST] From interface {request_id} to interface {send_request_to}")
                # rate = rclpy.Rate(3)
                # end = rclpy.Time.now() + rclpy.Duration(4.0)
                # while rclpy.Time.now() < end:
                #     self.interface_pub[send_request_to].publish()
                #     rate.sleep()
                
        # Process accept signals
        for response_id, response in self.share_responses.items():
            # Match response ID with the request ID
            request_id = self.signal_sent[response_id]
            
            # Do something with the request
            if response == self.SIGNAL_ACCEPT:
                self._team_exchange(response_id, request_id)
            else:
                pass

            # Clean up
            del self.share_requests[request_id]
            del self.signal_sent[response_id]

        self.share_responses = {}


    def _team_exchange(self, from_team_id, to_team_id, max_exchange=2):
        # Pick random robots to move from one team to the other? << start here
        # Or the closest? - this is better, but then we need to know the positions
        # Disallow someone to give away all their robots?
        ids = self.team_assignments.keys()
        random.shuffle(ids)
        transferred = []
        for id_ in ids:
            team_id = self.team_assignment[id_]
            if team_id == from_team_id:
                self.team_assignment[id_] = to_team_id
                self.team_info_pub.publish(to_team_id)
                transferred.append(id_)

            if len(transferred) == max_exchange:
                self.get_logger().info(f"[TRANSFER] From team {from_team_id} to team {to_team_id}: \
                    {transferred[0]}, {transferred[1]}")
                
                break


    def share_autonomous(self):
        # check which teams requested
        # check how many robots in each team
        # check how many the tasks need
        # distribute evenly but give higher priority to task with more robots
        for request in self.share_requests.items():
            if len(self.share_requests) > 0:
                request_id = random.choice(self.share_requests.keys())
                teams_available = range(self.num_teams)
                del teams_available[request_id]
                if len(teams_available) > 0:
                    send_request_to = random.choice(teams_available)

        self._team_exchange()
        return


def main(args=None):
    """
    Main function to initialize and run the RobotTeaming ROS2 node.
    """
    rclpy.init(args=args)
    robot_teaming_node = RobotTeaming()
    rclpy.spin(robot_teaming_node)
    robot_teaming_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()