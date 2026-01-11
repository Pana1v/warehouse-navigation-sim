#!/bin/bash

# 1. Kill Gazebo backend and frontend (Sim & Ignition)
pkill -9 -f "gz sim"
pkill -9 -f "ign gazebo" 
pkill -9 -f "ruby"

# 2. Kill ROS nodes and Launchers
pkill -9 -f "ros2 launch"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "rviz2"
pkill -9 -f "parameter_bridge"
pkill -9 -f "slam_toolbox"

# 3. Kill specific python nodes
pkill -9 -f "odom_to_tf.py"
pkill -9 -f "patrol_controller.py"
pkill -9 -f "simple_odometry.py"

echo "All specified processes have been killed."
