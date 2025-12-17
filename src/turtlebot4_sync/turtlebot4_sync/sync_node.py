import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from turtlebot4_custom_msg.msg import Turtlebot4Info, TeamInfo

import os
import yaml
import re
import numpy as np
import math


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

        # Set center of mill
        self.declare_parameter('center_x', 0)
        self.declare_parameter('center_y', 0) 
        
        self.center = np.array([
            int(self.get_parameter('center_x').value),
            int(self.get_parameter('center_y').value)])

        # Get robots in team
        sync_teams_path = os.path.join(get_package_share_directory(
            'turtlebot4_sync'), 'config', 'sync_teams.yaml')
        
        with open(sync_teams_path, 'r') as f:
            team_data = yaml.full_load(f)
        
        # First get the team that robot is in using its namespace
        for team, team_list in team_data['teams'].items():
            team_id = int(re.search(r'\d', team).group())
            if self.namespace[1:] in team_list:
                self.robot_team_id = team_id
                break
        
        robots_in_team = team_data['teams'][f'team{self.robot_team_id}']

        # Subscribe to info of teammates - positions used to compute avoidance force
        for namespace in robots_in_team:
            follower_namespace = f'/{namespace}'
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

        # Publisher for robot control commands
        self.control_publisher = self.create_publisher(TwistStamped, f'{self.namespace}/cmd_vel', 10)

        # Default swarm parameters
        v = 1.0
        radius = 1.
        
        # Movement constraints and interaction parameters
        self.min_v = 0.1
        self.max_v = 1.0
        self.min_k_radial = 1.0
        self.max_k_radial = 3.0

        # Milling forces
        self.k_radial = 1.0
        self.k_tangent = 1. # This should be stronger than k_radial to maintain the mill radius
        self.k_align = 0.2

        # Collision avoidance
        self.R_avoid = 0.5      # avoidance radius
        k_avoid = 0.5      # strength of repulsion

        # Init swarm parameters
        self.v = v
        self.radius = radius
        self.f_repulsion = k_avoid
        self.min_force = 0.5
    
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
        self.compute_forces()

    def update_parameters_callback(self, msg):
        """
        Callback function for updating the robot's behaviour parameters.
        
        Args:
            msg: A TeamInfo message containing the team's behaviour parameters.
        """
        self.v = min(self.max_v, max(self.min_v, msg.v))
        self.k_radial = min(self.max_k_radial, max(self.min_k_radial, msg.k_radial))
        
        print(f"Updated team parameters: v = {self.v}, k_radial = {self.k_radial}")
    
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
        # Preparations for force calculation and twist message
        twist = TwistStamped()
        Fx, Fy = self._quaternion_to_xy(self.orientation)
        F = np.array([Fx, Fy]) # Current force vector wrt. to world frame

        # -----------------------------------
        # Compute radial, tangent and align force based on current position and center of mill
        # -----------------------------------
        # Radial direction
        to_center = self.center - self.position
        dist = np.linalg.norm(to_center)
        radial_dir = to_center / dist if dist > 1e-6 else np.zeros(2) # Vector pointing directly to the center
        radial_force = self.k_radial * (dist - self.radius) * radial_dir

        # Tangential (rotate radial vector by 90°)
        tangent = np.array([-radial_dir[1], radial_dir[0]]) # Tangent vector
        tangent_force = self.k_tangent * tangent
        align_force = self.k_align * (F / np.linalg.norm(F)) # Force aligning to current force vector

        # -----------------------------------
        # Compute collision avoidance force
        # -----------------------------------
        # diff = self.robot_positions - self.position
        # distances = np.linalg.norm(diff, axis=1)

        # mask = (distances > 1e-6) & (distances < self.R_avoid)
        # if not np.any(mask):
        #     return np.zeros(2)

        # # inverse distance repulsion
        # dirs = diff[mask] / distances[mask][:, None]
        # weights = (self.R_avoid - distances[mask]) / self.R_avoid   # stronger when closer

        # # repel away from neighbors
        # repel = -np.sum(dirs * weights[:, None], axis=0)
        # avoid = self.f_repulsion * repel

        # Avoidance force calculation for each robot
        avoid = np.zeros(2)
        for namespace, pos in self.robot_positions.items():
            distance_robot, angle_to_robot = self._calculate_distance_and_angle(pos) #TODO check if this can be simplified
            if distance_robot > 0 and distance_robot < self.R_avoid:  # Avoid division by zero
                diff = pos - self.position
                dirs = diff / distance_robot
                weight = (self.R_avoid - distance_robot) / self.R_avoid   # stronger when closer

                # repel away from neighbors
                repel = dirs * weight
                avoid += self.f_repulsion * repel
                
                # Use Lennard-Jones potential for attraction and repulsion
                # v_to_follower = self.calculate_vector(distance_robot, angle_to_robot)
                # v_to_follower *= -1
                # ljf_follower = self.lennard_jones_potential(distance_follower, epsilon=self.follower_epsilon, sigma=self.follower_sigma, exponent=self.follower_exponent)
                # v_to_follower = self.get_unit_vector(v_to_follower) * ljf_follower
                # total_v_to_followers += v_to_follower
        # print("FORCES:", radial_force, tangent_force, align_force, avoid)
        # -----------------------------------
        # Combine forces
        # -----------------------------------
        total_force = radial_force + tangent_force + align_force + avoid # New force vector wrt. world frame
        
        # -----------------------------------
        # Normalize to constant speed
        # -----------------------------------
        norm = np.linalg.norm(total_force)
        # if norm < 1e-6:
        #     total_force = F
        #     norm = np.linalg.norm(total_force)
        
        new_F = total_force / norm * self.v # Normalised and multiplied with robot speed
        # v_total_force = -v_total_force  # Invert the force vector to get the direction to mov
        total_force_angle = math.atan2(new_F[1], new_F[0]) # Compute the rotation required wrt. world frame
        d_angle = total_force_angle - self._quaternion_to_yaw(self.orientation) # Rotation required wrt. current orientation
        d_angle = d_angle%(2*math.pi)
        if d_angle > math.pi:
            d_angle -= 2*math.pi

        if norm > self.min_force:
            if abs(d_angle) < math.pi/4:
                self.state = 'Move'
                twist.twist.linear.x = self.v
                twist.twist.angular.z = d_angle
            else:
                self.state = 'Rotate'
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = d_angle
        else:
            self.state = 'Stop'
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0


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
