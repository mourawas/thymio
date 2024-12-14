# EPFL - Mobile Robotics Project

This project, developed for the **Basics of Mobile Robotics** course (Fall 2024), demonstrates a fully functional system for controlling a **Thymio robot** in a dynamic environment. The system integrates the following modules:
- **Vision**: Processes camera images to localize the robot, detect the goal, and map obstacles using **AprilTags**.
- **Global Planning**: Calculates the optimal path from start to goal using **Dijkstra's algorithm** on an 8-connected grid.
- **Local Planning**: Generates motion commands to follow the global path while avoiding dynamic obstacles using proximity sensors and motion control strategies.
- **Filtering**: Combines odometry and camera data with an **Extended Kalman Filter (EKF)** to estimate the robot’s position and orientation accurately.

## Environment
The robot operates on a custom-designed map with:
- **Black cutouts** as static obstacles.
- A **red mark** indicating the goal.
- **AprilTags** for robot and corner detection.

## Demo
The project demonstrates a working system capable of navigating through a predefined environment while avoiding obstacles and reaching the goal. There are both predefined static obstacles and dynamic obstacles which can be placed anywhere around the robot. The robot can be kidnapped and will recompute its path correctly, and the camera can be obstructed after the initialization without affector the robot's behaviour.

## Group Members
- **Rocca Federico** (390233) - MSc Robotics  
- **Rashidi Mohammad Massi** (394309) - MSc CyberSecurity  
- **Rawas Mouhamad Bilal** (345489) - MSc Robotics  
- **Schär Mikaël Joël Michel** (325388) - MSc Robotics  

## Requirements
Install dependencies with:

```bash
pip install -r requirements.txt
