import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped
from turtlebot4_custom_msg.msg import Turtlebot4Info, TeamInfo

import os
import yaml
import re
import numpy as np
import math
import random


class RobotSync(Node):
    """
    A ROS2 Node for coordinating two robot teams in a synchronisation experiment.

    This node implements swarm milling behaviour around a center point. Parameters
    for the swarm behaviour (e.g. velocity) can be modified by a human operator.

    """

    def __init__(self):
        """
        Initialize the RobotSync node with default parameters and subscriptions.
        """
        super().__init__('robot_sync')

        # Get the robot's namespace from the ROS parameter server
        self.namespace = self.get_namespace()

        if self.namespace == "/":
            raise Exception("Robot namespace required")

        # Get robots in team
        sync_teams_path = os.path.join(get_package_share_directory(
            'turtlebot4_sync'), 'config', 'sync_teams.yaml')
        
        with open(sync_teams_path, 'r') as f:
            team_data = yaml.full_load(f)
        
        self.other_robot_team_id = 99 # Dummy variable
        # First get the team that robot is in using its namespace
        for team, team_list in team_data['teams'].items():
            team_id = int(re.search(r'\d', team).group())
            if self.namespace[1:] in team_list['namespace']:
                self.robot_team_id = team_id
            else:
                self.other_robot_team_id = team_id # Assuming only two teams
        
        robots_in_team = team_data['teams'][f'team{self.robot_team_id}']['namespace']
        if 'orbital_center' in team_data['teams'][f'team{self.robot_team_id}']:
            self.center = np.array([
                float(team_data['teams'][f'team{self.robot_team_id}']['orbital_center'][0]),
                float(team_data['teams'][f'team{self.robot_team_id}']['orbital_center'][1])
            ])
        else:
            self.center = np.array([0.0,0.0])

        if 'v' in team_data['teams'][f'team{self.robot_team_id}']:
            v = float(team_data['teams'][f'team{self.robot_team_id}']['v'])
        else:
            v = 0.5 #float(random.randint(1,10)/10)

        if 'radius' in team_data['teams'][f'team{self.robot_team_id}']:
            radius = float(team_data['teams'][f'team{self.robot_team_id}']['radius'])
        else:
            radius = 0.7 #float(random.randint(5,10)/10)

        # Subscribe to info of teammates - positions used to compute avoidance force
        for namespace in robots_in_team:
            follower_namespace = f'/{namespace}'
            if follower_namespace == self.namespace:
                continue

            # Turtlebot4info sub
            self.create_subscription(
                Turtlebot4Info, 
                f'{follower_namespace}/tb_info_topic', 
                self.create_follower_info_callback(follower_namespace), 
                10)

        # Subscribes to this robot's information
        self.create_subscription(Turtlebot4Info, f'{self.namespace}/tb_info_topic', self.update_robot_info_callback, 10)
        # TeamInfo sub
        self.create_subscription(TeamInfo, f'/team_info/team{self.robot_team_id}', self.update_parameters_callback, 10)
        # Other TeamInfo sub
        self.create_subscription(TeamInfo, f'/team_info/team{self.other_robot_team_id}', self.check_sync_callback, 10)

        # Publisher for robot control commands
        self.control_publisher = self.create_publisher(TwistStamped, f'{self.namespace}/cmd_vel', 10)
        self.team_info_publisher = self.create_publisher(TeamInfo, f'/team_info/team{self.robot_team_id}/out', 10) # send info to interface
        self.success_publisher = self.create_publisher(Int8, f'/team_info/sync_success', 10) # send info to interface
        self.init_interface_pub = 0

        # Default swarm parameters
        # v = 0.3
        # radius = 0.7
        robot_max_v = 0.25
        
        # Movement constraints and interaction parameters
        self.min_v = 0.1
        self.max_v = 1.0
        self.min_radius = 0.5
        self.max_radius = 1.0

        # Milling forces
        self.k_radial = 1.
        self.k_align = 0.1

        if self.robot_team_id == 0:
            self.k_tangent = 1. # This should be stronger than k_radial to maintain the mill radius
        else:
            self.k_tangent = -1.

        # Collision avoidance
        self.R_avoid = 0.5  # avoidance radius
        self.R_avoid_soft = 2*radius*np.sin(np.pi/len(robots_in_team))

        # k_avoid = 0.5      # strength of repulsion

        # Init swarm parameters
        self.v = v
        self.radius = radius
        self.v_input = v # input from interface
        self.radius_input = radius # input from interface
        self.robot_max_v = robot_max_v
        # self.f_repulsion = k_avoid
        self.min_force = 0.5
        self.n_robots = len(robots_in_team)
    
        self.position = np.array([0, 0])
        self.orientation = np.array([0, 0, 0, 0])
        self.robot_positions = {} # Store record of other robot positions
        self.robot_orientations = {} # Store record of other robot orientations

    def create_follower_info_callback(self, namespace):
        """
        Generates a callback function for handling updates to a follower's pose information.
        
        Args:
            namespace: The ROS namespace of the follower robot.
        
        Returns:
            A callback function that updates the internal state with the follower's position and orientation.
        """
        def follower_info_callback(msg):
            self.robot_positions[namespace] = np.array([msg.x, msg.y])
            self.robot_orientations[namespace] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  
        return follower_info_callback

    def update_robot_info_callback(self, msg):
        """
        Callback function for updating the robot's current position and orientation.
        
        Args:
            msg: A Turtlebot4Info message containing the robot's current pose.
        """
        self.position = np.array([msg.x, msg.y])
        self.orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        # Publish parameters to interface on start up
        # @TODO make this more robust
        if self.init_interface_pub < 20: # Set threshold to ensure update
            team_info = TeamInfo()
            team_info.v = self.v
            team_info.radius = self.radius
            self.team_info_publisher.publish(team_info)
            self.init_interface_pub += 1

        self.compute_forces()

    def update_parameters_callback(self, msg):
        """
        Callback function for updating the robot's behaviour parameters.
        
        Args:
            msg: A TeamInfo message containing the team's behaviour parameters.
        """
        if msg.v == 0:
            self.v_input = 0
        else:
            self.v_input = min(self.max_v, max(self.min_v, msg.v))
        self.radius_input = min(self.max_radius, max(self.min_radius, msg.radius))
        
        print(f"Updated team parameters: v = {self.v_input}, radius = {self.radius_input}")
    
    def check_sync_callback(self, msg):
        """
        Callback function to check synchronisation of both teams.
        
        Args:
            msg: A TeamInfo message containing the OTHER team's behaviour parameters.
        """
        other_v = msg.v
        other_radius = msg.radius

        msg = Int8()
        if abs(self.v - other_v) < 0.05 and abs(self.radius - other_radius) < 0.05:
            # synchronisation achieved
            msg.data = 1    
        else:
            msg.data = 0

        self.success_publisher.publish(msg)

    def _quaternion_to_yaw(self, quaternion):
        """
        Converts a quaternion orientation to a yaw angle.

        Args:
            quaternion: A list or numpy array containing the quaternion (x, y, z, w).
        
        Returns:
            The yaw angle in radians.
        """
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def _quaternion_to_xy(self, quaternion):
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return cosy_cosp, siny_cosp

    def _calculate_distance_and_angle(self, other_position):
        """
        Calculates the distance and angle to another position from the robot's current position.

        Args:
            other_position: The target position as a numpy array [x, y].
        
        Returns:
            A tuple (distance, angle) where distance is the Euclidean distance to the target
            and angle is the direction to the target in radians, relative to the robot's current orientation.
        """
        dx = other_position[0] - self.position[0]
        dy = other_position[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)

        follower_yaw = self._quaternion_to_yaw(self.orientation)
        angle_to_other = math.atan2(dy, dx) - follower_yaw
        angle_to_other = math.atan2(math.sin(angle_to_other), math.cos(angle_to_other))

        return distance, angle_to_other
    

    def compute_forces(self):
        """
        Implements an algorithm for milling behaviour around a center point while avoiding collisions with 
        other robots.

        This method calculates a centrifugal force around the center point and avoidance forces to other 
        robots, then publishes a Twist message to adjust the robot's velocity accordingly.
        """

        if self.v_input == 0:
            return

        # Adjust v and radius according to inputs from interface
        if self.v != self.v_input:
            diff = self.v - self.v_input
            self.v -= diff*0.1
        if self.radius != self.radius_input:
            diff = self.radius - self.radius_input
            self.radius -= diff*0.1

        # Preparations for force calculation and twist message
        twist = TwistStamped()
        Fx, Fy = self._quaternion_to_xy(self.orientation)
        F = np.array([Fx, Fy]) # Current force vector wrt. to world frame

        # -----------------------------------
        # Compute radial, tangent and align force based on current position and center of mill
        # -----------------------------------
        # Radial direction
        to_center = self.center - self.position
        dist = np.linalg.norm(to_center) # this is the true radius
        radial_dir = to_center / dist if dist > 1e-6 else np.zeros(2) # Vector pointing directly to the center
        radial_force = self.k_radial * (dist - self.radius) * radial_dir

        # Adjust k_radial to get closer to target radius
        r_e = 0.1
        if dist > self.radius + r_e:
            self.k_radial += 0.05
        elif dist < self.radius - r_e:
            self.k_radial -= 0.05

        self.k_radial = max(min(self.k_radial, 4.0), 0.1)
        self.radius_true = dist
        self.R_avoid_soft = 0.85*(2*self.radius_true*np.sin(np.pi/self.n_robots))
        # print("True radius:", dist, " / k_radial force:", self.k_radial)
        
        # Tangential (rotate radial vector by 90°)
        tangent = np.array([-radial_dir[1], radial_dir[0]]) # Tangent vector
        tangent_force = self.k_tangent * tangent
        align_force = self.k_align * (F / np.linalg.norm(F)) # Force aligning to current force vector
        
        # Combine forces
        total_force = radial_force + tangent_force + align_force # New force vector wrt. world frame

        v = self.v
        # Position robots equidistant on orbit
        # First find the robot in front
        closest_d = 1000
        closest_ns = None
        for namespace, pos in self.robot_positions.items():
            distance_robot, angle_to_robot = self._calculate_distance_and_angle(pos) #TODO check if this can be simplified
            if distance_robot > 0 and angle_to_robot < np.pi/4 and angle_to_robot > -np.pi/4:
                if distance_robot < closest_d:
                    closest_d = distance_robot
                    closest_ns = namespace
                
        # print("FORCES:", radial_force, tangent_force, align_force, avoid)

        # -----------------------------------
        # Normalize to constant speed
        # -----------------------------------
        norm = np.linalg.norm(total_force)
        # if norm < 1e-6:
        #     total_force = F
        #     norm = np.linalg.norm(total_force)
        
        new_F = total_force / norm# * self.v # Normalised and multiplied with robot speed
        # v_total_force = -v_total_force  # Invert the force vector to get the direction to mov
        total_force_angle = math.atan2(new_F[1], new_F[0]) # Compute the rotation required wrt. world frame
        d_angle = total_force_angle - self._quaternion_to_yaw(self.orientation) # Rotation required wrt. current orientation
        d_angle = d_angle%(2*math.pi)
        if d_angle > math.pi:
            d_angle -= 2*math.pi

        if norm > self.min_force:
            if abs(d_angle) < math.pi/4:
                self.state = 'Move'
                twist.twist.linear.x = v*self.robot_max_v
                twist.twist.angular.z = d_angle
            else:
                self.state = 'Rotate'
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = d_angle
        else:
            self.state = 'Stop'
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0

        # Check how close the robot in front is and apply speed/force restrictions
        if closest_d < self.R_avoid:
            twist.twist.linear.x = 0.0
            # if twist.twist.angular.z < 0:
            #     twist.twist.angular.z = 0.
        elif closest_d < self.R_avoid_soft:
            print("Too close to %s: %f < %f  "%(closest_ns, closest_d, self.R_avoid_soft))
            twist.twist.linear.x = twist.twist.linear.x*0.25
            # twist.twist.angular.z = twist.twist.angular.z*0.2


        # print(f'State: {self.state} / ', 'total_force:', new_F, ' / angle:', d_angle)

        # Publishing the twist message for movement control
        self.control_publisher.publish(twist)


def main(args=None):
    """
    Main function to initialize and run the RobotFlocking ROS2 node.
    """
    rclpy.init(args=args)
    robot_sync_node = RobotSync()
    rclpy.spin(robot_sync_node)
    robot_sync_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
