# MR_PathPlanner - APG-RRT Path Planner
This project was done as a part of the ME 5550 (Mobile Robotics) course at Northeastern University.

This project is an implementation inspired by the work of Z. Wang, P. Li, Z. Wang, and Z. Li, “APG-RRT: Sampling-Based Path Planning Method for Small Autonomous Vehicle in Closed Scenarios,” in IEEE Access, vol. 12, pp. 25731–25739, 2024, doi: 10.1109/ACCESS.2024.3359643
. Their research introduced the Adaptive Path Guided Rapidly-exploring Random Tree (APG-RRT), a variant of RRT designed to improve path quality and convergence in structured environments. My work builds upon this idea by implementing the algorithm in both 2D simulated environments and in a ROS2-based robotic simulation, while comparing its performance against the standard RRT.


* [Objective](#Objective)
* [Features](#Features)
* [Implementation](#Implementation)

## Objective:

The primary objective of this project was to develop and evaluate the APG-RRT algorithm as a more efficient alternative to standard RRT for autonomous mobile robot navigation. The aim was to generate smoother, shorter, and collision-free trajectories in structured environments such as corridors, bends, and narrow passages. Additionally, I sought to extend the algorithm from simple 2D Python simulations into a ROS2 framework using the TurtleBot3 robot in Gazebo.

## Features:
- The APG-RRT algorithm enhances standard RRT by combining guided exploration with adaptive sampling. It introduces preset guide paths tailored for common navigation challenges, such as long corridors, sharp bends, and U-turns, ensuring that tree expansion follows the structure of the environment.
- Sampling probabilities are dynamically adjusted based on the algorithm’s history, with successful expansions reinforcing guide paths and failed attempts reducing their weight. To ensure safe navigation, obstacle boundaries are expanded with a buffer to prevent near-collision trajectories.
- At the same time, the algorithm retains a degree of random exploration to avoid local minima, preserving the exploratory nature of RRT. The implementation also integrates seamlessly with ROS2, where planned paths can be published as topics and visualized in RViz.

    <img width="675" height="609" alt="image" src="https://github.com/user-attachments/assets/91f4d641-3cbe-4b3a-b119-ecfb2b17036b" />
         (APG-RRT Structure/Framework)

## Implementation:
### To run the APG_RRT path planner in ROS2:
- Build and source the workspace
- ros2 run path_planner apg_rrt_planner
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
- ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
- rviz2 (To visualize)

### To run the APG_RRT path planner in VSCODE:
- python3 apg_rrt.py
- python3 APG_RRT_COMPARISON.py (for the comparison plots)

## Results:

<img width="743" height="811" alt="image" src="https://github.com/user-attachments/assets/6b1a0beb-51d6-42a2-8f73-314f5bd32861" />

<img width="473" height="236" alt="image" src="https://github.com/user-attachments/assets/e5571c6f-d2e3-43cb-a05b-b7dcbf93e2d9" />


