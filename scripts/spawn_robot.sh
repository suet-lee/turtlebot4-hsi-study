# !/bin/bash

if ! [[ "$1" =~ ^[0-9]+$ ]]; then
    echo "Please provide a turtlebot ID."
    exit 1
fi

# Calculate the y-position with 0.3 spacing
y_position=$(echo "scale=1; $1 * 0.5" | bc)

# Spawn more follower robots
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py namespace:=turtlebot4_$1 x:=0.0 y:=$y_position &
# Activate gazebo_ros_bridge for each follower robot
ros2 run ros_gz_bridge parameter_bridge /model/turtlebot4_$1/turtlebot4/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V

wait