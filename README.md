# Warehouse Simulation

## Environment decision
AWS Robomaker Small Warehouse was too heavy, so switched to a lightweight custom warehouse in Gazebo Harmonic for faster load times.

## Highlights
1. Dynamic Obstacles and Avoidance
2. Added /scan as a source for the Obstacle Layer in the Global Planner as well, allowing global path also to respond to obstacles decreases navigation time.
3. Tuned to pass through narrow alleys
4. Fused Odometry through EKF
5. Finer costmap resolution and misc adjustments for close space navigation
6. Smoothened Path output from the Global Planner
7. Used a separate map and navigation world wherein the world had dynamic obstacles.


# Run Instructions: Warehouse Navigation Sim

## 1. Prerequisites
Ensure the following are installed and configured:

### Gazebo Harmonic
Verify you have Gazebo Harmonic installed:
```bash
gz sim --version
# Output should be >= 8.0.0 (Harmonic)
```

### Cyclone DDS
This project requires `Cyclone DDS` for reliable communication between ROS 2 and Gazebo.
1.  **Install** (if not present):
    ```bash
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    ```
2.  **Verify Configuration**:
    ```bash
    echo $RMW_IMPLEMENTATION
    # Output MUST be: rmw_cyclonedds_cpp
    ```
    If not, run:
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

---

## 2. Build the Project
Navigate to your workspace root and build the `bot_bringup` package.

```bash
cd ~/exp/del_ws
colcon build --symlink-install --packages-select bot_bringup
source install/setup.bash
```

*   `--symlink-install`: Allows you to edit Python scripts and config files (like Params/URDFs) without rebuilding.
*   `--packages-select`: Only builds the relevant package to save time.

---

## 3. Run the Simulation
Use the `bringup.launch.py` script to start the entire stack (Gazebo, Nav2, Robot).

```bash
# Terminal 1: Main Simulation Launch
ros2 launch bot_bringup bringup.launch.py
```

### What to Expect:
1.  **Gazebo** will open with the `simple_warehouse.sdf` world.
2.  **RViz2** will launch showing the map and robot.
3.  **Localization**: The robot should localize automatically (AMCL + EKF).

---

## 4. Restarting the Simulation (CRITICAL)
If you need to restart the simulation, please kill all lingering Gazebo and ROS processes first to avoid conflicts. The standard `Ctrl+C` often leaves background processes running.

```bash
bash kill_processes.sh
```

---

## 5. Operational Commands

### Teleoperate (Manual Control)
If you want to drive manually to test physics/mapping:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Send Navigation Goals
You can use the **2D Goal Pose** tool in RViz2 to send the robot to any location on the map.

## 5. Kill Processes before restart 
```bash
bash kill_processes.sh
```

## Videos

### Navigation
https://github.com/user-attachments/assets/a8ebebc5-76f5-4b1f-bd00-57da695df5e3

### Navigating Alleys
https://github.com/user-attachments/assets/748cc37b-8025-4c64-880f-8c5257e20247

### Smooth Paths
[smooth_paths.webm](https://github.com/user-attachments/assets/5cfe628b-35f5-495a-917f-1f007e22039b)

## Stress Test

### Placing a Pallete (here shelf) in path
[Screencast from 2026-01-11 09-24-29.webm](https://github.com/user-attachments/assets/9dfd5476-6ca9-459a-81ea-194ecd918747)

### Dynamic obstacles

### Test 1
https://github.com/user-attachments/assets/cc93f030-62dd-4eb3-bcb1-3099b67246f5

### Test 2
https://github.com/user-attachments/assets/73a7c295-08cc-41d9-82b0-7dd7345f964c
