# EPFL-MICRO452 - Mobile Robotics Project
Project for the Mobile Robotic course, fall semester 2024

## Group Members
Rocca Federico (390233) - First year MSc in Robotics at EPFL, previous degree BSc in Computer Engineering at Politecnico di Milano  
Rashidi Mohammad Massi () -  
Rawas Mouhamad Bilal () -  
Schär Mikaël Joël Michel (325388) - First year MSc in Robotics at EPFL, previous degree BSc in microengineering at EPFL

## Introduction
The aim of the project is to build a system able of controlling a [Thymio](https://www.thymio.org/) robot. This system shoud integrate all the main modules that are tipically found in a simple mobile robot:
- **Vision**
- **Global Navigation**
- **Local Navigation**
- **Filtering**

#### Demo

#### Environment
The environment where the robot has to navigate has been decided by us, and it consists of a white floor, the navigable space, where we placed some black cutouts, that represent permanent obstacles, while the goal position is indicated by a red mark placed on the floor. In order to detect the white Thymio robot in the white background we decided to place a blue and green marker over its top and the choice of a rectangular shape is needed for extracting the orientation, together with the position.

#### Features Implemented
- The **vision** module generates the map of the environment, starting from a camera image.
- The **vision** module also tracks the pose of the robot.
- The information from the **vision** is also used to detect and solve any kidnapping situation, by relocalizing the robot in the map
- The **global navigation** module, given the map of the environment, with a starting position and a goal, deigns the optimal plan using the Dijkstra algorithm.
- The Thymio control module generates the movement commands that allow the robot to move along the path from start to finish
- The **local navigation** module is used for avoiding the unexpected obstacles that might be detected along the path
- The **filtering** module implements a Kalman filter in order to better localize the robot. It predicts the robots position given the movements made and updates the prediction using data from the vision.

#### Explanation of Implemented Python Classes

#### Modules integration and Control Loop

#### Tuning the Parameters
