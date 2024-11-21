# EPFL-MICRO452 - Mobile Robotics Project
Project for the Mobile Robotic course, fall semester 2024

## Group Members
Rocca Federico (390233) - First year MSc in Robotics at EPFL, previous degree BSc in Computer Engineering at Politecnico di Milano  
Rashidi Mohammad Massi () -  
Rawas Mouhamad Bilal () -  
Schär Mikaël Joël Michel () -  

## Introduction
The aim of the project is to build a system able of controlling a [Thymio](https://www.thymio.org/) robot. This system shoud integrate all the main modules that are tipically found in a simple mobile robot:
- **Vision**
- **Global Navigation**
- **Local Navigation**
- **Filtering**

#### Environment
The environment where the robot has to navigate has been decided by us, and it consists of a white floor, the navigable space, where we placed some black cutouts, that represent permanent obstacles, while the goal position is indicated by a blue mark placed on the floor. In order to detect the white Thymio robot in the white background we decided to place a red marker over its top and the choice of a rectangular shape is needed for extracting the orientation, together with the position.

#### Features Implemented
- **Vision**: generates the map of the environment, starting from a camera image, and tracks the pose of the robot
- **Global Navigation**: given the map of the environment, with a starting position and a goal, deigns the optimal plan using the Dijkstra algorithm
- **Local Navigation**:

#### Explanation of Implemented Python Classes