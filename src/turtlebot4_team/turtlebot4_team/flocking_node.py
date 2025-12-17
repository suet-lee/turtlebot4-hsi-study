import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped
from turtlebot4_custom_msg.msg import Turtlebot4Info  # Import custom message

import numpy as np
import math
import os
import yaml
import re

class RobotFlocking(Node):
    """
    A ROS2 Node for coordinating robot flocking behavior, leveraging a custom Turtlebot4Info message.

    This node implements a flocking algorithm where a group of robots (followers) aim to maintain
    a formation while following a leader. It subscribes to information about the leader and other
    followers' positions and publishes velocity commands to maintain the desired formation.
    """
    def __init__(self):
        """
        Initialize the RobotFlocking node with default parameters and subscriptions.
        """
        super().__init__('robot_flocking')

        # Get the robot's namespace from the ROS parameter server
        self.namespace = self.get_namespace()

        # Get robots in team
        share_teams_path = os.path.join(get_package_share_directory(
            'turtlebot4_team'), 'config', 'share_teams.yaml')
        
        with open(share_teams_path, 'r') as f:
            team_data = yaml.full_load(f)
        
        robot_list = {}
        for team_, team_list in team_data['teams'].items():
            for robot in team_list:
                robot_list[robot] = int(team_[-1]) # Assume team IDs < 10 (single char)

        self.follower_positions = {}
        self.follower_orientations = {}
        self.follower_team = {}

        # Subscribe to info of teammates - positions used to compute avoidance force
        for namespace, team_id in robot_list.items():
            follower_namespace = f'/{namespace}'
            if follower_namespace != self.namespace:
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
            else:
                self.robot_team_id = team_id

        # self.get_logger().info(f"Maximum number of teammates: {self.max_teammates}")
        self.get_logger().info(f"Team ID: {self.robot_team_id}")

        # Subscribes to all leader robot's information
        for team, leader_namespace in team_data['leaders'].items():
            team_id = int(re.search(r'\d', team).group())
            self.create_subscription(
                Turtlebot4Info, 
                f'/{leader_namespace}/tb_info_topic', 
                self.create_leader_info_callback(team_id), 
                10)
        
        # Subscribes to this robot's information
        self.create_subscription(Turtlebot4Info, f'{self.namespace}/tb_info_topic', self.update_robot_info_callback, 10)
        # Subscribes to this robot's team info
        self.create_subscription(Int8, f'{self.namespace}/team', self.update_robot_team_callback, 10)

        # Publisher for robot control commands
        self.control_publisher = self.create_publisher(TwistStamped, f'{self.namespace}/cmd_vel', 10)
        
        # Initialize robot and leader positions and orientations
        self.robot_position = np.array([0, 0])
        self.robot_orientation = np.array([0, 0, 0, 0])
        self.leader_position = np.array([0, 0])
        self.leader_orientation = np.array([0, 0, 0, 0])

        # Movement constraints and interaction parameters
        self.max_forward_velocity = 1.0
        self.max_rotation_velocity = 0.5

        self.leader_sigma = 1.3
        self.leader_epsilon = 0.3
        self.leader_exponent = 1

        self.follower_sigma = 1.1
        self.follower_epsilon = 0.1#0.5
        self.follower_exponent = 2

        self.flock_distance_threshold = 1.0
        self.min_force = 0.5

        self.state = 'GO'

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
            self.follower_orientations[namespace] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  
        return follower_info_callback

    def update_robot_info_callback(self, msg):
        """
        Callback function for updating the robot's current position and orientation.
        
        Args:
            msg: A Turtlebot4Info message containing the robot's current pose.
        """

        self.robot_position = np.array([msg.x, msg.y])
        self.robot_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
 

    def create_follower_team_callback(self, namespace):
        """
        Generates a callback function for handling updates to a follower's team info.
        
        Args:
            namespace: The ROS namespace of the follower robot.
        
        Returns:
            A callback function that updates the internal state with the follower's team info.
        """
        def follower_team_callback(msg):
            self.follower_team[namespace] = msg.data # use this information to determine which robots to account for in flocking computation

        return follower_team_callback

    def update_robot_team_callback(self, msg):
        """
        Callback function for updating the robot's team info.
        
        Args:
            msg: A Turtlebot4Info message containing the robot's tam info.
        """
        
        self.robot_team_id = msg.data # this is assigned by teaming_node, here we update so the robot knows which leader to follow


    #TODO is it useful to store information for all leaders?
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
            if leader_id != self.robot_team_id:
                return

            self.leader_position = np.array([msg.x, msg.y])
            self.leader_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.calculate_forces()

        return leader_info_callback

    def quaternion_to_yaw(self, quaternion):
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

    def calculate_vector(self, distance, angle_radians):
        """
        Calculates a 2D vector from a distance and angle.

        Args:
            distance: The magnitude of the vector.
            angle_radians: The direction of the vector in radians.
        
        Returns:
            A numpy array representing the vector.
        """
        x = distance * np.cos(angle_radians)
        y = distance * np.sin(angle_radians)
        return np.array([x, y])
    
    def calculate_distance_and_angle(self, other_position):
        """
        Calculates the distance and angle to another position from the robot's current position.

        Args:
            other_position: The target position as a numpy array [x, y].
        
        Returns:
            A tuple (distance, angle) where distance is the Euclidean distance to the target
            and angle is the direction to the target in radians, relative to the robot's current orientation.
        """
        dx = other_position[0] - self.robot_position[0]
        dy = other_position[1] - self.robot_position[1]
        distance = math.sqrt(dx**2 + dy**2)

        follower_yaw = self.quaternion_to_yaw(self.robot_orientation)
        angle_to_other = math.atan2(dy, dx) - follower_yaw
        angle_to_other = math.atan2(math.sin(angle_to_other), math.cos(angle_to_other))

        return distance, angle_to_other
    
    def lennard_jones_potential(self, distance, epsilon=1.0, sigma=0.5, exponent=6):
        """
        Calculates the Lennard-Jones potential energy between two particles.

        Parameters:
        - distance: The distance between the particles.
        - epsilon: The depth of the potential well.
        - sigma: The finite distance at which the inter-particle potential is zero.

        Returns:
        - The potential energy between the particles.
        """
        return 4 * epsilon * ((sigma / distance)**(2*exponent) - (sigma / distance)**exponent)

    def get_unit_vector(self, vector):
        """
        Returns the unit vector in the same direction as the input vector.

        Args:
            vector: The input vector as a numpy array.
        
        Returns:
            The unit vector in the same direction as the input vector.
        """
        return vector / np.linalg.norm(vector)

    def calculate_forces(self):
        """
        Adapts the flocking algorithm to follow the leader while avoiding collisions with followers.

        This method calculates the forces exerted by the leader and the followers on this robot, then
        publishes a Twist message to adjust the robot's velocity accordingly.
        """
        # Preparations for force calculation and twist message
        twist = TwistStamped()
        distance_to_leader, angle_to_leader = self.calculate_distance_and_angle(self.leader_position)

        # Initialize vectors and calculate force towards the leader
        v_to_leader = np.zeros(2)
        total_v_to_followers = np.zeros(2)

        # Force calculation for the leader
        if distance_to_leader > 0:  # Avoid division by zero
            v_to_leader = self.calculate_vector(distance_to_leader, angle_to_leader)

            if distance_to_leader < self.flock_distance_threshold:

                # If leader is close, use Lennard-Jones potential for attraction and repulsion
                v_to_leader *= -1
                ljf_leader = self.lennard_jones_potential(distance_to_leader, epsilon=self.leader_epsilon , sigma=self.leader_sigma, exponent=self.leader_exponent)
                v_to_leader = self.get_unit_vector(v_to_leader) * ljf_leader
            
            # else:
                # If leader is far, move towards the leader using the direct vector to the leader
                # v_to_leader = v_to_leader # (could be removed)

        # Force calculation for each follower
        for namespace, team_id in self.follower_team.items():
            # Check if follower belongs to the same team #TODO maybe this check not needed
            # if team_id != self.robot_team_id:
            #     continue

            pos = self.follower_positions[namespace]
            distance_follower, angle_to_follower = self.calculate_distance_and_angle(pos)
            if distance_follower > 0:  # Avoid division by zero
                # Use Lennard-Jones potential for attraction and repulsion
                v_to_follower = self.calculate_vector(distance_follower, angle_to_follower)
                v_to_follower *= -1
                ljf_follower = self.lennard_jones_potential(distance_follower, epsilon=self.follower_epsilon, sigma=self.follower_sigma, exponent=self.follower_exponent)
                v_to_follower = self.get_unit_vector(v_to_follower) * ljf_follower
                total_v_to_followers += v_to_follower

        # for pos in self.follower_positions.values():
        #     distance_follower, angle_to_follower = self.calculate_distance_and_angle(pos)
        #     if distance_follower > 0:  # Avoid division by zero
        #         # Use Lennard-Jones potential for attraction and repulsion
        #         v_to_follower = self.calculate_vector(distance_follower, angle_to_follower)
        #         v_to_follower *= -1
        #         ljf_follower = self.lennard_jones_potential(distance_follower, epsilon=self.follower_epsilon, sigma=self.follower_sigma, exponent=self.follower_exponent)
        #         v_to_follower = self.get_unit_vector(v_to_follower) * ljf_follower
        #         total_v_to_followers += v_to_follower

        # Total force calculation and twist message population
        v_total_force = v_to_leader + total_v_to_followers
        # v_total_force = -v_total_force  # Invert the force vector to get the direction to mov
        total_force = np.linalg.norm(v_total_force)
    

        total_force_angle = math.atan2(v_total_force[1], v_total_force[0])

        # transform angle from quaternion to yaw
        leader_yaw = self.quaternion_to_yaw(self.leader_orientation)
        robot_yaw = self.quaternion_to_yaw(self.robot_orientation)


        # angle difference between leader and robot
        leader_align_angle = math.atan2(math.sin(leader_yaw - robot_yaw), math.cos(leader_yaw - robot_yaw))
        
        if total_force > self.min_force:
            # Flock with leader
            if v_total_force[0] < 0:
                # Rotate to move towards the vector pointing backwards
                self.state = 'Flock rotate'
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = total_force_angle
            else:
                self.state = 'Flock Move'
                twist.twist.linear.x = total_force
                twist.twist.angular.z = total_force_angle

        elif abs(leader_align_angle) > 0.1:
            self.state = 'Align'
            # align with leader
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = leader_align_angle
        else:
            # stop
            self.state = 'Stop'
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0


        print(self.state, 'total_force:', total_force, 'angle:', total_force_angle)



        # Catch up if total force


        # if abs(total_force[0]) > self.min_force:
        #     if total_force[0] < 0:
        #         # Rotate to move towards the vector pointing backwards
        #         print('STATE: ROTATE', 'pos:', self.robot_position, 'angle:', angle)
        #         twist.twist.linear.x = 0.0
        #         if abs(angle) > 0.15:
        #             twist.twist.angular.z = angle
        #         else:
        #             twist.twist.angular.z = 0.0
        #     else:
        #         # Flock with leader that is moving

        #         print('STATE: FLOCK (MOVING_FORWARD)', 'pos:', self.robot_position)
        #         twist.twist.linear.x = min(np.linalg.norm(total_force), self.max_forward_velocity)
        #         twist.twist.angular.z = angle
        # # elif total_force[0] < -self.min_force:
        # #     print('STATE: FLOCK (MOVING_BACK)')
        # #     twist.twist.linear.x = max(-np.linalg.norm(total_force), -self.max_forward_velocity)
        # #     twist.twist.angular.z = -angle
        # elif abs(math.atan2(math.sin(leader_yaw - robot_yaw), math.cos(leader_yaw - robot_yaw))) > 0.15:
        #     # Align with leader that is not moving
        #     print('STATE: FLOCK (ALIGNING)', 'angle:', angle, math.atan2(math.sin(leader_yaw - robot_yaw), math.cos(leader_yaw - robot_yaw)))
        #     twist.twist.linear.x = 0.0
        #     twist.twist.angular.z = math.atan2(math.sin(leader_yaw - robot_yaw), math.cos(leader_yaw - robot_yaw))
        # else:
        #     print('STATE: STOP', 'pos:', self.robot_position)
        #     twist.twist.linear.x = 0.0
        #     twist.twist.angular.z = 0.0




        # Publishing the twist message for movement control
        self.control_publisher.publish(twist)



def main(args=None):
    """
    Main function to initialize and run the RobotFlocking ROS2 node.
    """
    rclpy.init(args=args)
    robot_flocking_node = RobotFlocking()
    rclpy.spin(robot_flocking_node)
    robot_flocking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
