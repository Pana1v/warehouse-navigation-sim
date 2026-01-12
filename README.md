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
