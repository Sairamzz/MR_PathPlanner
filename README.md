# MR_PathPlanner - APG-RRT Path Planner
This project was done as a part of the ME 5550 (Mobile Robotics) course at Northeastern University.

Implementing this work: Z. Wang, P. Li, Z. Wang and Z. Li, "APG-RRT: Sampling-Based Path Planning Method for Small Autonomous Vehicle in Closed Scenarios," in IEEE Access, vol. 12, pp. 25731-25739, 2024, doi: 10.1109/ACCESS.2024.3359643.
keywords: {Motion planning;Path planning;Heuristic algorithms;Roads;Kinematics;Vehicle dynamics;Urban areas;Autonomous vehicles;Autonomous vehicles;path planning;motion planning;sampling based path planning;RRT;path smoothing},



* [Objective](#Objective)
* [Features](#Features)
* [Implementation](#Implementation)

## Objective:
## Features:
## Implementation:
## Results:

### To run the APG_RRT path planner in ROS2:
- Build and source the workspace
- ros2 run path_planner apg_rrt_planner
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
- ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
- rviz2 (To visualize)

### To run the APG_RRT path planner in VSCODE:
- python3 apg_rrt.py
- python3 APG_RRT_COMPARISON.py (for the comparison plots)


