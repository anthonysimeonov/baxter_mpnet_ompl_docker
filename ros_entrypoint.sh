#!/bin/bash
set -e

# setup ros environment
source "/root/catkin_ws/devel/setup.bash"

# Setting environment variable for ROS
ROS_IP=$(ip addr show eth0 | grep -Po 'inet \K[\d.]+')
export ROS_IP=$ROS_IP

# eval  "roslaunch baxter_gazebo baxter_world.launch"
# eval "roslaunch baxter_moveit_experiments baxter_moveit.launch"

# TODO: A exit sequence for identifying failed initialization or broken Gazebo
exec "$@"
