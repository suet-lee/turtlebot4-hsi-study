source /opt/ros/jazzy/setup.bash

MODE=$1
TB_IP=$2

if [ "$MODE" == "pc" ]; then
    export ROS_DISCOVERY_SERVER=";127.0.0.1:11811"

elif [ "$MODE" == "tb" ]; then
    if [ "$TB_IP" == "" ]; then
        echo "Usage: source $0 tb [IP address]"
        return 1 2>/dev/null || exit 1
    fi
    export ROS_DISCOVERY_SERVER="${TB_IP}:11811;127.0.0.1:11811"
else
    echo "Usage: source $0 [pc | tb]"
    return 1 2>/dev/null || exit 1
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=0

echo "[$MODE] ROS_DISCOVERY_SERVER: \"${ROS_DISCOVERY_SERVER}\""
echo "[$MODE] ROS_SUPER_CLIENT: ${ROS_SUPER_CLIENT}"
echo "[$MODE] ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

ros2 daemon stop; ros2 daemon start