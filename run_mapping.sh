#!/bin/bash

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo "=========================================="
    echo "  🛑 CAUGHT SIGNAL - CLEANING UP ZOMBIES"
    echo "=========================================="
    
    # 1. Kill ROS Launch
    pkill -TERM -P $$

    # 2. Kill Gazebo & Bridge (The usual suspects)
    echo "Killing Gazebo and Bridges..."
    pkill -9 -f "gz sim"
    pkill -9 -f "ruby"
    pkill -9 -f "parameter_bridge"
    
    # 3. Kill Python Nodes
    echo "Killing Python Nodes..."
    pkill -9 -f "simple_odometry.py"
    pkill -9 -f "robot_state_publisher"
    pkill -9 -f "slam_toolbox"
    pkill -9 -f "rviz2"
    
    # 4. Final Sweep
    echo "Final Sweep..."
    # Only kill python3 processes that look like ROS nodes to avoid collateral damage
    pkill -9 -f "ros2/launch"
    
    echo "✅ Cleanup Complete."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

echo "=========================================="
echo "  🚀 STARTING MAPPING WITH AUTO-CLEANUP"
echo "=========================================="

# Build first to be sure
colcon build --packages-select bot_bringup --symlink-install
source install/setup.bash

# Run the launch file
ros2 launch bot_bringup mapping.launch.py &

# Wait for the launch process to finish (or for us to get Ctrl+C)
PID=$!
wait $PID
