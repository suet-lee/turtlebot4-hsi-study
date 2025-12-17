import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8
from turtlebot4_custom_msg.msg import Turtlebot4Info, TaskInfo

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


    def __init__(self):
        """
        Initialize the RobotTeaming node with default parameters and subscriptions.
        """
        super().__init__('task_manager')

        # Assume a NxN square grid, where N is odd and the center cell is inactive
        self.declare_parameter('zone_gridsize_n', 3) 
        self.zone_gridsize_n = self.get_parameter('zone_gridsize_n').value

        #TODO set in config file
        self.declare_parameter('arena_w', 7)
        self.arena_w = self.get_parameter('arena_w').value

        # Number of concurrently active zones
        self.declare_parameter('max_active_zones', 4)
        self.max_active_zones = self.get_parameter('max_active_zones').value

        self.zone_parameters = {}
        self.zone_status = {}
        self.task_requirement = {}
        self.task_progress = {}
        self.center_cell = None
        self.task_points = 0

        self.init_zones()

        self.zone_publisher = {}

        # Publish task info for all zones (for visual display)
        for k in self.zone_parameters.keys():
            self.zone_publisher[k] = self.create_publisher(TaskInfo, f'/zone{k}/task_info', 10)

        self.assign_tasks()

        self.leader_positions = {}
        self.leader_in_zone = {}
        self.follower_positions = {}
        self.follower_team = {}

        self.interface_publisher = {}

        # Get leader namespace
        share_teams_path = os.path.join(get_package_share_directory(
            'turtlebot4_team'), 'config', 'share_teams.yaml')
        
        with open(share_teams_path, 'r') as f:
            team_data = yaml.full_load(f)
        
        robot_list = {}
        for team_, team_list in team_data['teams'].items():
            team_id = int(re.search(r'\d', team_).group())
            for robot in team_list:
                robot_list[robot] = team_id #int(team_[-1])

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


    def init_zones(self):
        """
        Initialises the zones which are parameterised by the corner points.
        """

        # Divide the arena by gridsize, assume all cells are equal size and square
        # Parameterise zones in CW order starting from top left
        no_cells = math.floor(self.arena_w/self.zone_gridsize_n)
        cell_w = self.arena_w/no_cells
        center_cell = math.floor(no_cells/2)

        k = 0
        for i in range(no_cells+1):
            for j in range(no_cells+1):
                if i == center_cell and j == center_cell: # skip center cell
                    self.center_cell = k
                    continue

                x0 = i*cell_w
                x1 = (i+1)*cell_w
                y0 = j*cell_w
                y1 = (j+1)*cell_w
                
                self.zone_parameters[k] = {'x0':x0,'x1':x1,'y0':y0,'y1':y1}
                self.zone_status[k] = self.ZONE_INACTIVE
                self.task_requirement[k] = 0
                self.task_progress[k] = 1

                k += 1


    def assign_tasks(self, skip_zones=[]):
        active_zones = 0
        skip_zones = skip_zones
        # Loop over zones, check if active, publish zone info
        for zone, status in self.zone_status.items():
            if status == self.ZONE_ACTIVE:
                active_zones += 1
                skip_zones.append(zone)
            
            msg = TaskInfo()
            msg.id = zone
            msg.status = status
            msg.robots_available = 99 # dummy value
            msg.robots_required = self.task_requirement[zone]
            msg.progress = self.task_progress[zone]
            self.zone_publisher[zone].publish(msg)

        max_requirement = 7
        while active_zones < self.max_active_zones:
            # Randomly assign new active zone
            zones = list(self.zone_status.keys())
            new_active = random.choice(zones)
            if new_active in skip_zones:
                continue

            self.zone_status[new_active] = self.ZONE_ACTIVE
            new_requirement = random.randrange(3, max_requirement+1) # can also create a list of possible requirements e.g. [3,3,3,7]
            if new_requirement == max_requirement:
                max_requirement = 5
            self.task_requirement[new_active] = new_requirement
            self.task_progress[new_active] = 1

            # Publish new zone info
            msg = TaskInfo()
            msg.id = new_active
            msg.status = self.ZONE_ACTIVE
            msg.robots_available = 99 # dummy value
            msg.robots_required = self.task_requirement[new_active]
            msg.progress = 1
            self.zone_publisher[new_active].publish(msg)

            active_zones += 1
            skip_zones.append(new_active)


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
            self.follower_team[namespace] = msg.id

        return follower_team_callback

    
    # delta tolerance parameter
    def _check_in_zone(self, zone_id, x_pos, y_pos, delta=0):
        params = self.zone_parameters[zone_id]
        if x_pos < params['x0']-delta or x_pos > params['x1']+delta or y_pos < params['y0']-delta or y_pos > params['y1']+delta:
            return False
        return True

    def _compute_leader_zone(self, leader_id):
        x_pos, y_pos = self.leader_positions[leader_id]

        for zone_id, params in self.zone_parameters.items():
            # check if inside zone parameters
            if self._check_in_zone(zone_id, x_pos, y_pos):
                self.leader_in_zone[leader_id] = zone_id
                return zone_id
            

    # Check if a leader is in an active zone:
    # if occupied, check if task requirements are met
    # if requirements are met, progress the task
    # publish task info
    def check_leader_zone(self, leader_id):
        x_pos, y_pos = self.leader_positions[leader_id]
        
        # First check init
        if leader_id not in self.leader_in_zone:
            current_zone = self._compute_leader_zone(leader_id)
        # Check if it's in the same zone as previous
        else:
            zone_id = self.leader_in_zone[leader_id]
            # Could technically simplify but trying to reduce computation (does it though?)
            if not self._check_in_zone(zone_id, x_pos, y_pos):
                current_zone = self._compute_leader_zone(leader_id)
            else:
                current_zone = zone_id

        # Check if zone is active
        if self.zone_status[current_zone] == self.ZONE_INACTIVE:
            msg = TaskInfo()
            msg.id = current_zone
            msg.status = self.ZONE_INACTIVE
            msg.robots_available = 0
            msg.robots_required = 0
            msg.progress = 1
            self.interface_publisher.publish(msg)
            return
        
        # Zone is active, so check if task requirement is met
        reqs = self.task_requirement[current_zone]
        progress = self.task_progress[current_zone]

        team_members = 0
        for fnamespace, team_id in self.follower_team.items():
            if team_id != leader_id:
                continue

            x_pos, y_pos = self.follower_positions[fnamespace]
            if self._check_in_zone(self, current_zone, x_pos, y_pos, delta=0.2):
                team_members += 1

        if team_members >= reqs['robots']:
            progress += 1

        msg = TaskInfo()
        msg.id = current_zone
        msg.status = self.ZONE_ACTIVE
        msg.robots_available = team_members
        msg.robots_required = reqs['robots']
        msg.progress = progress
        self.interface_publisher.publish(msg)

        if progress >= 100:
            self.zone_status[current_zone] = self.ZONE_INACTIVE
            self.task_requirement[current_zone] = 0
            self.task_progress[current_zone] = 1
            self.assign_tasks(skip_zones=[current_zone])

        
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

        


        
        
        


