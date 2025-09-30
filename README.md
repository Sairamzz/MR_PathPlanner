# MR_PathPlanner - APG-RRT Path Planner
This project was done as a part of the ME 5550 (Mobile Robotics) course at Northeastern University.

* [About the Project](#about)

## About the Project



### To run the APG_RRT path planner in ROS2:
- Build and source the workspace
- ros2 run path_planner apg_rrt_planner
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
- ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
- rviz2 (To visualize)

### To run the APG_RRT path planner in VSCODE:
- python3 apg_rrt.py
- python3 APG_RRT_COMPARISON.py (for the comparison plots)
