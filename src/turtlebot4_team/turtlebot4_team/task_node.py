import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8, Int16
from turtlebot4_custom_msg.msg import Turtlebot4Info, TaskInfo, TrialInfo

import random
import yaml
import os
import re
import math
import numpy as np

class TaskManager(Node):
    """
    A ROS2 Node for managing tasks.

    This node manages the activation and progression of tasks. It tracks if an operator is
    currently in an active task zone and whether their team has enough robots to progress
    the task.

    """

    ZONE_INACTIVE = 0
    ZONE_ACTIVE = 1

    POINTS_SMALL = 100
    POINTS_BIG = 150

    def __init__(self):
        """
        Initialize the RobotTeaming node with default parameters and subscriptions.
        """
        super().__init__('task_manager')

        # Assume a NxN square grid, where N is odd and the center cell is inactive
        self.declare_parameter('zone_gridsize_n', 3) 
        self.zone_gridsize_n = self.get_parameter('zone_gridsize_n').value

        #TODO set in config file
        self.declare_parameter('arena_w', 6.8)
        self.arena_w = self.get_parameter('arena_w').value
        self.arena_h = self.arena_w # Assume square

        # Number of concurrently active zones
        self.declare_parameter('max_active_zones', 4)
        self.max_active_zones = self.get_parameter('max_active_zones').value

        self.declare_parameter('seed', 10)
        self.random_seed = self.get_parameter('seed').value

        self.trial_state = 0 #TODO define as class variable

        self.zone_parameters = {}
        self.zone_status = {}
        self.task_requirement = {}
        self.task_progress = {}
        self.center_cell = None
        self.task_points = {}
        self.task_points_pub = {}

        self.init_zones()

        self.zone_publisher = {}

        # Publish task info for all zones (for visual display)
        for k in self.zone_parameters.keys():
            self.zone_publisher[k] = self.create_publisher(TaskInfo, f'/zone{k}/task_info', 10)

        self.control_sub = self.create_subscription(
                TrialInfo,
                '/trial_info',
                self.create_trial_callback,
                10
            )

        random.seed(self.random_seed) #  10 for active, 20 for autonomous
        self.assign_tasks([self.center_cell])

        self.leader_positions = {}
        self.leader_in_zone = {}
        self.follower_positions = {}
        self.follower_team = {}
        self.robots_available = {} # Count of robots in an active zone
        self.zone_occupation = {}

        self.interface_publisher = {}
        self.which_zone_publisher = {}

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
        
        # Subscribe to info of robots
        for namespace, team_id in robot_list.items():
            follower_namespace = f'/{namespace}'
            # Turtlebot4info sub
            self.create_subscription(
                Turtlebot4Info, 
                f'{follower_namespace}/tb_info_topic', 
                self.create_follower_info_callback(follower_namespace), 
                    10)
            # Sub to team info topic
            self.create_subscription(
                Int8,
                f'{follower_namespace}/team', 
                self.create_follower_team_callback(follower_namespace), 
                    10)
            # Init follower team assignment
            self.follower_team[follower_namespace] = team_id
            # Publish zone that robot is currently in
            self.which_zone_publisher[follower_namespace] = self.create_publisher(
                Int8,
                f'{follower_namespace}/zone', 
                10
            )
        
        # Subscribes to all leader robot's information
        for team, leader_namespace in team_data['leaders'].items():
            team_id = int(re.search(r'\d', team).group())
            self.create_subscription(
                Turtlebot4Info, 
                f'/{leader_namespace}/tb_info_topic', 
                self.create_leader_info_callback(team_id), 
                    10)
            
            # Publisher for task info to each leader/interface
            self.interface_publisher[team_id] = self.create_publisher(TaskInfo, f'/interface{team_id}/task_info', 10)
            self.robots_available[team_id] = 0
            
            # Publish zone that leader is currently in
            self.which_zone_publisher[f'l{team_id}'] = self.create_publisher(
                Int8,
                f'{leader_namespace}/zone',
                10
            )
            
            # init task points
            self.task_points[team_id] = 0
            self.task_points_pub[leader_namespace] = self.create_publisher(
                Int16,
                f'/task_points/team{team_id}',
                10
            )


    def init_zones(self):
        """
        Initialises the zones which are parameterised by the corner points.
        """

        # Divide the arena by gridsize, assume all cells are equal size and square
        # Parameterise zones in CW order starting from top left
        no_cells = self.zone_gridsize_n #math.floor(self.arena_w/self.zone_gridsize_n)
        cell_w = self.arena_w/no_cells
        self.center_cell = math.floor(no_cells*no_cells/2)
        
        k = 0
        for i in range(no_cells):
            for j in range(no_cells):
                if i == self.center_cell and j == self.center_cell: # skip center cell
                    continue

                x0 = i*cell_w
                x1 = (i+1)*cell_w
                y0 = j*cell_w
                y1 = (j+1)*cell_w

                # Offset by center coord
                x0 -= self.arena_w/2
                x1 -= self.arena_w/2
                y0 -= self.arena_h/2
                y1 -= self.arena_h/2
                
                self.zone_parameters[k] = {'x0':x0,'x1':x1,'y0':y0,'y1':y1}
                self.zone_status[k] = self.ZONE_INACTIVE
                self.task_requirement[k] = 0
                self.task_progress[k] = 1

                k += 1

    def assign_tasks(self, skip_zones=[]):
        active_zones = 0
        skip_zones.append(self.center_cell)

        # Assume there are two requirement values
        low_req = 4
        high_req = 6
        low_req_count = 0
        high_req_count = 0

        # Loop over zones, check if active
        for zone, status in self.zone_status.items():
            if status == self.ZONE_ACTIVE:
                active_zones += 1
                skip_zones.append(zone)
                if self.task_requirement[zone] == low_req:
                    low_req_count += 1
                if self.task_requirement[zone] == high_req:
                    high_req_count += 1
            
            # msg = TaskInfo()
            # msg.id = zone
            # msg.status = status
            # msg.robots_available = 99 # dummy value
            # msg.robots_required = self.task_requirement[zone]
            # msg.progress = self.task_progress[zone]
            # self.zone_publisher[zone].publish(msg)

        while active_zones < self.max_active_zones:
            # Randomly assign new active zone
            zones = list(self.zone_status.keys())
            new_active = random.choice(zones)
            if new_active in skip_zones:
                continue

            self.zone_status[new_active] = self.ZONE_ACTIVE
            if high_req_count < math.ceil(self.max_active_zones/2):
                new_requirement = high_req
                high_req_count += 1
            else:
                new_requirement = low_req
                low_req_count += 1

            self.task_requirement[new_active] = new_requirement
            self.task_progress[new_active] = 1

            # Publish new zone info
            # msg = TaskInfo()
            # msg.id = new_active
            # msg.status = self.ZONE_ACTIVE
            # msg.robots_available = 99 # dummy value
            # msg.robots_required = self.task_requirement[new_active]
            # msg.progress = 1
            # self.zone_publisher[new_active].publish(msg)

            active_zones += 1
            skip_zones.append(new_active)
        

    def create_trial_callback(self, msg):
        
        self.trial_state = msg.state
        if self.trial_state == 0:
            # idle
            print("Trial idle")
            return
        
        elif self.trial_state == 1:
            # running
            print("Trial running")
            for zone, status in self.zone_status.items():            
                new_msg = TaskInfo()
                new_msg.id = zone
                new_msg.status = status
                new_msg.robots_available = 99 # dummy value - the real number is published to the interface topic
                new_msg.robots_required = self.task_requirement[zone]
                new_msg.progress = int(self.task_progress[zone])
                self.zone_publisher[zone].publish(new_msg)

            for leader_id in [0,1]:
                new_msg = TaskInfo()
                try:
                    # leader_in_zone may not be initialised
                    zone = self.leader_in_zone[leader_id]
                    new_msg.id = zone
                    new_msg.status = self.zone_status[zone]
                    new_msg.robots_available = self.robots_available[zone]
                    new_msg.robots_required = self.task_requirement[zone]
                    new_msg.progress = int(self.task_progress[zone])
                    
                    msg = Int8()
                    msg.data = zone
                    self.which_zone_publisher[f'l{leader_id}'].publish(msg)
                except Exception as e:
                    new_msg.id = -1
                    new_msg.status = self.ZONE_INACTIVE
                    new_msg.robots_available = 0
                    new_msg.robots_required = 0
                    new_msg.progress = 1
                finally:
                    self.interface_publisher[leader_id].publish(new_msg)

            for namespace, pub in self.task_points_pub.items():
                leader_id = int(re.search(r'\d', namespace).group())
                msg = Int16()
                msg.data = self.task_points[leader_id]
                pub.publish(msg)

            for fnamespace, team_id in self.follower_team.items():
                if fnamespace in self.zone_occupation:
                    msg = Int8()
                    msg.data = self.zone_occupation[fnamespace]
                    self.which_zone_publisher[fnamespace].publish(msg)

        elif self.trial_state == 2:
            # pause
            print("Trial paused")
            return
               

    def create_leader_info_callback(self, leader_id):
        """
        Generates a callback function for handling updates to the leader's pose information.
        
        Args:
            namespace: The ID of the leader robot.
        
        Returns:
            A callback function that updates the internal state with the leader's position and orientation.
            Checks if the leader_id matches the robot's current team ID.
        """
        def leader_info_callback(msg):
            self.leader_positions[leader_id] = np.array([msg.x, msg.y])
            self.check_leader_zone(leader_id)

        return leader_info_callback
    

    def create_follower_info_callback(self, namespace):
        """
        Generates a callback function for handling updates to a follower's pose information.
        
        Args:
            namespace: The ROS namespace of the follower robot.
        
        Returns:
            A callback function that updates the internal state with the follower's position and orientation.
        """
        def follower_info_callback(msg):
            self.follower_positions[namespace] = np.array([msg.x, msg.y])
            
        return follower_info_callback


    def create_follower_team_callback(self, namespace):
        """
        Generates a callback function for handling updates to a follower's team info.
        
        Args:
            namespace: The ROS namespace of the follower robot.
        
        Returns:
            A callback function that updates the internal state with the follower's team info.
        """
        def follower_team_callback(msg):
            self.follower_team[namespace] = msg.data

        return follower_team_callback
    
    # delta tolerance parameter
    def _check_in_zone(self, zone_id, x_pos, y_pos, delta=0):
        if zone_id == -1:
            return False
        
        params = self.zone_parameters[zone_id]
        if math.isnan(x_pos) or math.isnan(x_pos):
            return False

        try:
            if x_pos < params['x0']-delta or x_pos > params['x1']+delta or y_pos < params['y0']-delta or y_pos > params['y1']+delta:
                return False
            else:
                return True
        except Exception as e:
            return False
        

    def _compute_leader_zone(self, leader_id):
        x_pos, y_pos = self.leader_positions[leader_id]
        
        for zone_id, params in self.zone_parameters.items():
            # check if inside zone parameters
            if self._check_in_zone(zone_id, x_pos, y_pos):
                self.leader_in_zone[leader_id] = zone_id
                self.zone_occupation[f'l{leader_id}'] = zone_id
                return zone_id

        self.leader_in_zone[leader_id] = -1    
        return -1
    

    # Check if a leader is in an active zone:
    # if occupied, check if task requirements are met
    # if requirements are met, progress the task
    # publish task info
    def check_leader_zone(self, leader_id):
        x_pos, y_pos = self.leader_positions[leader_id]
        
        if leader_id not in self.leader_in_zone:
            # Do first zone check (initialise)
            current_zone = self._compute_leader_zone(leader_id)
        else:
            # Check if it's in the same zone as previous
            zone_id = self.leader_in_zone[leader_id]
            if not self._check_in_zone(zone_id, x_pos, y_pos):
                current_zone = self._compute_leader_zone(leader_id)
            else:
                current_zone = zone_id

        if current_zone == -1: # Leader outside arena
            print(f"Leader {leader_id} outside arena")
            return
        
        # Check if zone is active
        if self.zone_status[current_zone] == self.ZONE_INACTIVE:
            return
        
        print(f"Leader {leader_id} in active zone")
        
        # Zone is active, so check if task requirement is met
        robot_req = self.task_requirement[current_zone]
        progress = self.task_progress[current_zone]
        
        team_members = 0
        for fnamespace, team_id in self.follower_team.items():
            if team_id != leader_id:
                continue
            
            try:
                x_pos, y_pos = self.follower_positions[fnamespace]
                if self._check_in_zone(current_zone, x_pos, y_pos, delta=0.3):
                    team_members += 1
            except Exception as e:
                pass

        # print(f"Leader {leader_id} in zone {current_zone}, robot count = {team_members}")

        if team_members >= robot_req:
            print("Progress ",progress)
            progress += 0.25
            self.task_progress[current_zone] = progress

        self.robots_available[current_zone] = team_members
        if progress >= 100:
            # Success! Award points
            if robot_req == 6:
                self.task_points[leader_id] += self.POINTS_BIG
            else:
                self.task_points[leader_id] += self.POINTS_SMALL
            # reset zone on completion
            self.zone_status[current_zone] = self.ZONE_INACTIVE
            self.task_requirement[current_zone] = 0
            self.task_progress[current_zone] = 1
            self.assign_tasks(skip_zones=[current_zone]) # Make sure the same zone is not reactivated

        
def main(args=None):
    """
    Main function to initialize and run the TaskManager ROS2 node.
    """
    rclpy.init(args=args)
    task_manager_node = TaskManager()
    rclpy.spin(task_manager_node)
    task_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        


        
        
        


