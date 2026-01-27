import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8, Int16
from turtlebot4_custom_msg.msg import ShareRequest, TrialInfo, DistanceInfo, TaskInfo

from hsi_trial_control import TrialStates as tstate

import random
import yaml
import os
import re
import time
from operator import itemgetter

class RobotTeaming(Node):
    """
    A ROS2 Node for coordinating the robot teams.

    This node implements team assignment and re-assignment which enables robots to be shared 
    between teams. It subscribes to information communicated from human operators (via an 
    interface) to determine team (re-)assignments. It also manages the behavioural parameters 
    (such as velocity, density and orientation) for all robots in a given team.

    """

    TASK_ACTIVE_SHARE = 1
    TASK_AUTONOMOUS_SHARE = 2 # Passive share

    SIGNAL_DEFAULT = 0
    SIGNAL_REQUEST = 1
    SIGNAL_CANCEL = 2
    SIGNAL_ACCEPT = 3
    SIGNAL_ACCEPT_DELAY = 4
    SIGNAL_SEND_RESOLVED = 5
    SIGNAL_RECV_RESOLVED = 6
    # SIGNAL_TIMEOUT = 5 #TODO not used - remove

    INTF_IDLE = 0
    INTF_RQT_SEND = 1
    INTF_RQT_CANCEL = 2
    INTF_RQT_RECEIVE = 3 # New request received by interface2
    INTF_RQT_ACCEPT = 4 # Request accepted by interface2
    INTF_RQT_DELAY = 5 # Accepted with delay

    POINTS_SHARE = 40

    def __init__(self):
        """
        Initialize the RobotTeaming node with default parameters and subscriptions.
        """
        super().__init__('robot_teaming')

        self.declare_parameter('task_id', self.TASK_ACTIVE_SHARE) 
        self.task_id = self.get_parameter('task_id').value

        self.share_requests = {}
        self.share_cancels = {}
        self.share_responses = {}
        self.signal_sent = {}
        self.team_assignments = {}
        self.distance_score = {}
        self.equal_teams = True # On init teams are equal size

        self.intf_send_state = {} # State of sending channel for interface
        self.intf_recv_state = {} # State of recieving channel for interface

        # Hardcode the sender-receiver pairs, can be extended for more participants
        self.sender_receiver = {
            0: 1,
            1: 0
        } 
        
        self.request_count = {} # ID to assign to requests
        self.share_points = {} # points for sharing robots
        self.share_points_pub = {}
        # self.interface_state = {}

        # Publishers
        self.interface_pub = {}
        self.interface_state_pub = {}
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
        if self.task_id == self.TASK_ACTIVE_SHARE:
            self.get_logger().info(f"Task type: Active share")
        elif self.task_id == self.TASK_AUTONOMOUS_SHARE:
            self.get_logger().info(f"Task type: Autonomous share")

        # Subscribe to receive interface messages
        for team, leader_namespace in team_data['leaders'].items():
            team_id = int(re.search(r'\d', team).group())
            # Receive incoming requests
            self.create_subscription(
                ShareRequest,
                f'/interface{team_id}/share_request/to_node',
                self.create_share_request_callback(team_id),
                10
            )

            # Publishers to send to user interface
            self.interface_pub[team_id] = self.create_publisher(
                ShareRequest,
                f'/interface{team_id}/share_request/to_interface',
                10
            )

            self.intf_send_state[team_id] = self.INTF_IDLE
            self.intf_recv_state[team_id] = self.INTF_IDLE
            self.request_count[team_id] = 0
            self.interface_state_pub[team_id] = self.create_publisher(
                Int8,
                f'/interface{team_id}/state',
                10
            )

            # Create team info publisher for each leader
            # leader_namespace = f'/leader{i}'
            self.team_assignments[leader_namespace] = team_id
            ns_ = f'/{leader_namespace}'
            self.team_info_pub[ns_] = self.create_publisher(
                Int8,
                f'{leader_namespace}/team', 
                10
            )
            # msg = Int8()
            # msg.data = team_id
            # self.team_info_pub[leader_namespace].publish(msg) # Set default team ID of the leader to its ID

            # Subscriber for task info for leader
            self.leader_zone_info = {}
            self.create_subscription(
                    TaskInfo, 
                    f'/interface{team_id}/task_info', 
                    self.create_leader_zone_callback(team_id),
                    10
                )
            
            self.share_points[team_id] = 0
            self.share_points_pub[leader_namespace] = self.create_publisher(
                Int16,
                f'/share_points/team{team_id}',
                10
            )
        
        # Create team info publisher for each follower
        # Create subscriber to distance information
        for namespace, team_id in robot_list.items():
            follower_namespace = f'/{namespace}'
            self.team_assignments[namespace] = team_id
            self.team_info_pub[follower_namespace] = self.create_publisher(
                Int8,
                f'{follower_namespace}/team', 
                10
            )

            self.create_subscription(
                DistanceInfo,
                f'{follower_namespace}/distance_info',
                self.create_distance_info_callback(follower_namespace),
                10
            )
    
        # Listens to msgs from control node  - publishes team info
        self.create_subscription(
                TrialInfo,
                f'/trial_info',
                self.control_callback,
                10
            )

############################################################################

    # Computes the distance score for each robot according to:
    # S = 2*d_F - d_T, d_F=distance from, d_T=distance to other team
    def create_distance_info_callback(self, namespace):

        def distance_info_callback(msg):
            from_team = self.team_assignments[namespace[1:]]

            if from_team == 0: #TODO remove this hack
                d_F = msg.distance0
                d_T = msg.distance1
            else:
                d_F = msg.distance1
                d_T = msg.distance0

            self.distance_score[namespace] = 2*d_F-d_T

        return distance_info_callback
        

    def control_callback(self, msg):
        if msg.state == tstate.s_TRIAL_IDLE:
            # Pause sharing
            return
        elif msg.state == tstate.s_TRIAL_RUNNING:
            # Publish team info
            for ns, team_id in self.team_assignments.items():
                if "leader" in ns:
                    continue

                if ns[0] != "/":
                    ns = "/"+ns

                msg = Int8()
                msg.data = team_id
                self.team_info_pub[ns].publish(msg)

            for interface1_id, state in self.intf_send_state.items():
                interface2_id = self.sender_receiver[interface1_id]
                if state == self.INTF_RQT_SEND:
                    msg = ShareRequest()
                    msg.sender_id = interface1_id
                    msg.request_id = self.request_count[interface1_id]
                    msg.data = self.SIGNAL_REQUEST
                    self.interface_pub[interface2_id].publish(msg)

                if state == self.INTF_RQT_CANCEL:
                    msg = ShareRequest()
                    msg.sender_id = interface1_id
                    msg.data = self.SIGNAL_CANCEL
                    self.interface_pub[interface2_id].publish(msg)

                if state == self.INTF_IDLE: # send idle state to interface / request resolved
                    msg = Int8()
                    msg.data = self.SIGNAL_SEND_RESOLVED
                    self.interface_state_pub[interface1_id].publish(msg)

            for interface1_id, state in self.intf_recv_state.items():

                if state == self.INTF_IDLE: # send idle state to interface / accept resolved
                    msg = Int8()
                    msg.data = self.SIGNAL_RECV_RESOLVED
                    self.interface_state_pub[interface1_id].publish(msg)

            for namespace, pub in self.share_points_pub.items():
                leader_id = int(re.search(r'\d', namespace).group())
                msg = Int16()
                msg.data = self.share_points[leader_id]
                pub.publish(msg)

    def create_share_request_callback(self, interface1_id):
        """
        Generates a callback function for handling signals from a user interface.
        
        Args:
            namespace: The ID of the interface.
        
        Returns:
            A callback function that triggers the correct robot sharing mechanism depending on the signal content.
        """
        def share_request_callback(msg):
            # interface2_id = self.sender_receiver[interface1_id]
            intf_state = self.intf_send_state[interface1_id]
            if msg.data == self.SIGNAL_REQUEST: # Process share request from interface - sends it to other participant
                # print('req received', interface1_id)
                if intf_state == self.INTF_IDLE:
                    print('its a new one', self.request_count[interface1_id]+1)
                    self.intf_send_state[interface1_id] = self.INTF_RQT_SEND
                    # self.share_requests[send_to] = self.request_count
                    self.request_count[interface1_id] += 1 # This is sent to the other interface

                    if self.task_id == self.TASK_AUTONOMOUS_SHARE:
                        return self.share_autonomous()

                return
            
            elif msg.data == self.SIGNAL_CANCEL:
                # print('req cancelled', interface1_id)
                if intf_state == self.INTF_RQT_SEND:
                    self.intf_send_state[interface1_id] = self.INTF_RQT_CANCEL
                    # del self.share_requests[send_to]
                return
            
            elif msg.data == self.SIGNAL_SEND_RESOLVED and intf_state == self.INTF_RQT_CANCEL:
                # print('req resolved', interface1_id)
                self.intf_send_state[interface1_id] = self.INTF_IDLE
                return
            
            elif msg.data == self.SIGNAL_ACCEPT:
                # print('its accepting', interface1_id)
                self.intf_recv_state[interface1_id] = self.INTF_RQT_ACCEPT
                # self.share_responses[interface_id] = self.SIGNAL_ACCEPT
                self.process_signals()

            elif msg.data == self.SIGNAL_ACCEPT_DELAY:
                # print('its accepting wi delay', interface1_id)
                self.intf_recv_state[interface1_id] = self.INTF_RQT_DELAY
                # self.share_responses[interface_id] = self.SIGNAL_ACCEPT_DELAY

  
        return share_request_callback
    

    def create_leader_zone_callback(self, team_id):

        def leader_zone_callback(msg):
            self.leader_zone_info[team_id] = {
                'r_av': msg.robots_available,
                'r_req': msg.robots_required,
                'progress': msg.progress
            }

        return leader_zone_callback

    def process_signals(self):
        
        # Process accept signals
        for interface1_id in self.intf_recv_state.keys():
            state = self.intf_recv_state[interface1_id]
            if state != self.INTF_RQT_ACCEPT:
                continue

            team_count = self._count_teams()
            # Minimum robot limit, do not share
            # if interface1_id in team_count and team_count[interface1_id] == 4:
            #     continue

            # Match response ID with the request ID
            interface2_id = self.sender_receiver[interface1_id]

            # Do something with the request
            if self.equal_teams:
                max_exchange = 1
                self.equal_teams = False
            else:
                max_exchange = 2

            self._team_exchange(interface1_id, interface2_id, max_exchange) # interface1 accepted the exchange
            self.intf_recv_state[interface1_id] = self.INTF_IDLE # reset receive state
            self.intf_send_state[interface2_id] = self.INTF_IDLE # reset send state
            

    def _team_exchange(self, from_team_id, to_team_id, max_exchange=2):
        team_count = self._count_teams()
        if team_count[from_team_id] > 4:
            scores = {}
            for ns, score in self.distance_score.items():
                if self.team_assignments[ns[1:]] == from_team_id: #TODO make sure dict keys use consistent namespace wi/ or w/o slash
                    scores[ns[1:]] = score
            
            # Sorted from highest to lowest
            sorted_scores = {k: v for k, v in sorted(scores.items(), key=lambda item: item[1], reverse=True)}
            # Get the top N to exchange
            top = dict(sorted(sorted_scores.items(), key=itemgetter(1), reverse=True)[:max_exchange])       
            print('exchange  ',top)
            for ns, d in top.items():
                self.team_assignments[ns] = to_team_id
        
        self.intf_send_state[0] = self.INTF_IDLE # reset receive state
        self.intf_send_state[1] = self.INTF_IDLE


    def _count_teams(self):
        counts = {}
        for ns, team_id in self.team_assignments.items():
            if team_id in counts:
                counts[team_id] += 1
            else:
                counts[team_id] = 0
        return counts

    def share_autonomous(self):
        # Check the zone requirements for leaders
        # Allow share if a leader zone requires more robots, and the second leader is in a low requirement zone
        # If it's a draw, ignore the request
        if len(self.leader_zone_info) < 2:
            return
        
        team_count = self._count_teams()
        if team_count[0] == 5:
            max_exchange = 1
        else:
            max_exchange = 2
        
        # leader 0 doesn't have enough, leader 0 has < leader 1, and leader 1 has more than needed
        if self.intf_send_state[0] == self.INTF_RQT_SEND and \
            self.leader_zone_info[0]['r_req'] > team_count[0] and \
            self.leader_zone_info[0]['r_req'] > self.leader_zone_info[1]['r_req'] and \
            team_count[1] > self.leader_zone_info[1]['r_req']:

            print("sharing from 1 to 0")
            # share_from 1 to 0
            if team_count[1] > 4:
                self._team_exchange(1, 0, max_exchange)
                self.intf_send_state[0] = self.INTF_IDLE # reset receive state
                self.intf_send_state[1] = self.INTF_IDLE

        print(team_count, self.intf_send_state[1] == self.INTF_RQT_SEND and \
            self.leader_zone_info[1]['r_req'] > team_count[1] and \
            self.leader_zone_info[1]['r_req'] > self.leader_zone_info[0]['r_req'] and \
            team_count[0] > self.leader_zone_info[0]['r_req'])
        # the other way around
        if self.intf_send_state[1] == self.INTF_RQT_SEND and \
            self.leader_zone_info[1]['r_req'] > team_count[1] and \
            self.leader_zone_info[1]['r_req'] > self.leader_zone_info[0]['r_req'] and \
            team_count[0] > self.leader_zone_info[0]['r_req']:
            print("sharing from 0 to 1")
            # share_from 0 to 1
            if team_count[0] > 4:
                self._team_exchange(0, 1, max_exchange)
                self.intf_send_state[1] = self.INTF_IDLE # reset send state
                self.intf_send_state[0] = self.INTF_IDLE
            

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