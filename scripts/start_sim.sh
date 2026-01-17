#!/bin/bash

world=hangar 
rviz=false
namespace=$1

if [ "$namespace" == "" ]; then
    namespace=turtlebot4_0
fi


# Start the simulation and spawn leader robot
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py rviz:=$rviz world:=$world namespace:=$namespace &
# Activate gazebo_ros_bridge for the leader robot
ros2 run ros_gz_bridge parameter_bridge /model/$namespace/turtlebot4/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V &

ros2 launch turtlebot4_broadcast super_broadcaster_launch.py env:=sim

wait