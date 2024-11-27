# EPFL-MICRO452 - Mobile Robotics Project
Project for the Mobile Robotic course, fall semester 2024

## Group Members
Rocca Federico (390233) - First year MSc in Robotics at EPFL, previous degree BSc in Computer Engineering at Politecnico di Milano  
Rashidi Mohammad Massi () -  
Rawas Mouhamad Bilal () -  
Schär Mikaël Joël Michel (325388) - First year MSc in Robotics at EPFL, previous degree BSc in microengineering at EPFL

## Introduction
The aim of the project is to build a system able of controlling a <a href="https://www.thymio.org/" target="_blank">Thymio</a> robot. This system shoud integrate all the main modules that are tipically found in a simple mobile robot:
- **Vision**
- **Global Navigation**
- **Local Navigation**
- **Filtering**

#### Demo

#### Environment
The environment where the robot has to navigate has been designed by us, and it consists of a white floor, the navigable space, where we placed some black cutouts, that represent permanent obstacles, while the goal position is indicated by a red mark placed on the floor. In order to detect the white Thymio robot in the white background we decided to place a blue and green marker over its top and the choice of a rectangular shape is needed for extracting the orientation, together with the position.  
<p align="center">
    <img src="images/env_image.png" width="400">
</P>
We also attached 4 <a href="https://april.eecs.umich.edu/software/apriltag" target="_blank">AprilTags</a> on the 4 corners of the environment in order to straighten the image that the camera records and generate precise position and orientation measurements based on that image.  
<p align="center">
    <img src="images/tag.png" width="400">
</p>

#### Features Implemented
- The **vision** module generates the map of the environment, starting from a camera image.
- The **vision** module also tracks the pose of the robot.
- The information from the **vision** is also used to detect and solve any kidnapping situation, by relocalizing the robot in the map
- The **global navigation** module, given the map of the environment, with a starting position and a goal, deigns the optimal plan using the Dijkstra algorithm.
- The Thymio control module generates the movement commands that allow the robot to move along the path from start to finish
- The **local navigation** module is used for avoiding the unexpected obstacles that might be detected along the path
- The **filtering** module implements a Kalman filter in order to better localize the robot. It predicts the robots position given the movements made and updates the prediction using data from the vision.

#### Explanation of Implemented Python Classes
- **localPlanning.py**: An important part in robot's navigation is the avoidance of unexpected obstacles that might be detecte while moving from the start position to the goal, following the global plan. The process of local planning consists of detecting the obstacles using the robots sensors and designing a more or less efficient plan to get around it, avoiding collisions, and get back on the predetermined global plan.  
    The Thymio robot features 5 horizontal proximity sensors it its front part (see Thyimio cheat sheet snippet) that can be used to detect obstacles using infrared technology. The range of values that the sensors return is [0, 4300], ranging from nothing detected to something detected at minimum distance from the sensor, and the updates come at a frequency of 10Hz.
    <p align="center">
        <img src="images/thymio_cheat_sheet1.png" width="800">
    </p>
    It is possible to map the readings from the proximity sensors to the real world distance measurements in order to tune the local avoidance matrix. In order to do so, many measurements were taken with an obstacle at a known distance, for multiple distances, and then a linear interpolation allowed to find the function $f(sensor reading) = distance = {\alpha} * sensor_reading + {\betha}$ that maps sensor readings to obstacle distance.  
    The mapping has been calculated to be: $distance = n * sensor_reading + m$
    The following table shows the measurements that allowed to reconstruct the mapping:

    | Distance | Sensor Reading (average value) |
    | --- | ---: |
    | 5 | 12343 |
    | 10 | 123414 |
    | 20 | 324123 |


    The python file contains the <code>LocalPlanning</code> class that implements the local avoidance modules. It presents two methods:
    - <code>is_local_planning(prox_horizontal)</code>: takes the readings from the horizontal proximity sensors and checks if their value is higher than a threshold

#### Modules integration and Control Loop

#### Tuning the Parameters
