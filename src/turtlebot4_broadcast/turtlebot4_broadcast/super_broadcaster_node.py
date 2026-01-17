import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from turtlebot4_custom_msg.msg import Turtlebot4Info  # import custom message
try:
    from mocap4r2_msgs.msg import RigidBodies #, RigidBody
except:
    pass
    
from geometry_msgs.msg import PoseArray

import re
import os
import yaml

class SuperBroadcaster(Node):

    def __init__(self):
        super().__init__('super_broadcaster_node')

        self.declare_parameter('env', 'qualisys') 
        self.env = self.get_parameter('env').get_parameter_value().string_value
        
        if self.env not in ['qualisys', 'vicon', 'sim']:
            raise Exception('Environment must be: [ qualisys | vicon | sim ]')

        self.tb_namespaces = {}
        self.info_publishers = {}
        namespace_data = {}
        
        if self.env == 'qualisys':
            #TODO nicer way to do this?
            # This is a yaml file matching namespaces to rigid body names assigned in Qualisys
            rb_names_path = os.path.join(get_package_share_directory(
                'turtlebot4_broadcast'), 'config', 'rigid_body_names.yaml')
            
            with open(rb_names_path, 'r') as f:
                namespace_data = yaml.full_load(f)

            try:
                self.create_subscription(RigidBodies, '/rigid_bodies', self.qualisys_callback, 10)
            except:
                raise Exception("mocap4r2 packages missing")

        # elif self.env == 'vicon': #TODO add vicon

        elif self.env == 'sim':
            spawn_path = os.path.join(get_package_share_directory(
                'turtlebot4_broadcast'), 'config', 'sim_spawn.yaml')
            
            with open(spawn_path, 'r') as f:
                robot_list = yaml.full_load(f)
            
            for ns in robot_list['namespaces']:
                namespace_data[ns] = ns
                pose_topic = f'/model/{ns}/turtlebot4/pose'
                self.create_subscription(PoseArray, pose_topic, self.create_sim_callback(ns), 10)
        
        for namespace, label in namespace_data.items():
            info_topic = f'/{namespace}/tb_info_topic'
            self.tb_namespaces[label] = namespace
            self.info_publishers[label] = self.create_publisher(Turtlebot4Info, info_topic, 10)
        

    def qualisys_callback(self, rigid_bodies_msg): # callback real env

        for rigid_body in rigid_bodies_msg.rigidbodies:
            # Get the namespace corresponding to the rigid body name
            rb_name = rigid_body.rigid_body_name
            namespace = self.tb_namespaces[rb_name]

            # fill with the data coming from Qualysis tracking system
            info_msg = Turtlebot4Info()
            info_msg.x = rigid_body.pose.position.x
            info_msg.y = rigid_body.pose.position.y
            info_msg.orientation.x = rigid_body.pose.orientation.x
            info_msg.orientation.y = rigid_body.pose.orientation.y
            info_msg.orientation.z = rigid_body.pose.orientation.z
            info_msg.orientation.w = rigid_body.pose.orientation.w
            
            if 'leader' in namespace:
                info_msg.role = "leader"
            else:
                info_msg.role = "follower"

            if rb_name in self.info_publishers:
                self.info_publishers[rb_name].publish(info_msg)
            else:
                print(f"Rigid body not in config: {rb_name}")


    def create_sim_callback(self, namespace):

        def sim_callback(pose_msg): # callback sim env
            info_msg = Turtlebot4Info()
            # check for a number in the namespace and use as id
            if namespace == '/':
                info_msg.id = 0
            elif 'turtlebot4' in namespace:
                info_msg.id = int(namespace.split('_')[1])
            else:
                try:
                    info_msg.id = int(re.search(r'\d', namespace).group())
                except:
                    info_msg.id = -1

            if 'leader' in namespace:
                info_msg.role = "leader"
            else:
                info_msg.role = "follower"

            # take first item in array
            data = pose_msg.poses[0]

            # fill with the data coming from Ignition gazebo
            info_msg.x = data.position.x
            info_msg.y = data.position.y
            info_msg.orientation.x = data.orientation.x
            info_msg.orientation.y = data.orientation.y
            info_msg.orientation.z = data.orientation.z
            info_msg.orientation.w = data.orientation.w

            self.info_publishers[namespace].publish(info_msg)
        
        return sim_callback


def main(args=None):
    rclpy.init(args=args)
    super_broadcaster_node = SuperBroadcaster()
    rclpy.spin(super_broadcaster_node)
    super_broadcaster_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
