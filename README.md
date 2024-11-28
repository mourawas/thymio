# EPFL-MICRO452 - Mobile Robotics Project
Project for the Mobile Robotic course, fall semester 2024

## Group Members
**Rocca Federico** (390233) - First year MSc in Robotics at EPFL, previous degree BSc in Computer Engineering at Politecnico di Milano  
**Rashidi Mohammad Massi** () -  
**Rawas Mouhamad Bilal** () -  
**Schär Mikaël Joël Michel** (325388) - First year MSc in Robotics at EPFL, previous degree BSc in Microengineering at EPFL

## Introduction
The aim of the project is to build a system able of controlling a <a href="https://www.thymio.org/" target="_blank">Thymio</a> robot. This system shoud integrate all the main modules that are tipically found in a simple mobile robot:
- **Vision**
- **Global Navigation**
- **Local Navigation**
- **Filtering**

The robot should will be placed in an **environment** (explained later) where there are some **permanent obstacles**. Given a **camera view** of the environment, a **global plan** should be generated, starting from the Thymio's position, leading to the goal and avoiding the permanent obstacles. Some **random obstacles** may be introduced in the environment during the movement of the robot from start to goal, and it should be able to avoid them. The robot should be localized both using the camera image, that can be obstructed on purpose from time to time, and the odometry, by fusing them with the **filtering**.

### Demo
Here's a demo of the system working:  
<p align="center">
  <video width="640" height="480" controls>
    <source src="video.mov" type="video/mp4">
  </video>
</p>

### Environment
The environment that the robot has to navigate has been designed by us, and it consists of a **white floor**, the traversable space, where we placed some **black cutouts**, that represent permanent obstacles, while the goal position is indicated by a **red mark** placed on the floor.  
<p align="center">
    <img src="images/env_image.png" width="400">
</P>
In order to detect the white Thymio robot in the white background we decided to place an <a href="https://april.eecs.umich.edu/software/apriltag" target="_blank">AprilTag</a> on its top. It is used both for detecting the robot's position and orientation.
We also attached 4 AprilTags on the 4 corners of the environment in order to straighten the image that the camera records and generate precise measurements based on that image.  
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
- **localPlanning.py**: An important part in robot's navigation is the avoidance of unexpected obstacles that might be detected while moving from the start position to the goal, following the global plan. The process of local planning consists of detecting the obstacles using the robots sensors and designing a more or less efficient plan to get around it, avoiding collisions, and get back on the predetermined global plan.  
    The Thymio robot features 5 horizontal proximity sensors it its front part (see Thyimio cheat sheet snippet) that can be used to detect obstacles using infrared technology. The range of values that the sensors return is [0, 4300], ranging from nothing detected to something detected at minimum distance from the sensor, and the updates come at a frequency of 10Hz.
    <p align="center">
        <img src="images/thymio_cheat_sheet1.png" width="800">
    </p>
    It is possible to map the readings from the proximity sensors to the real world distance measurements in order to tune the local avoidance matrix. In order to do so, many measurements were taken with an obstacle at a known distance, for multiple distances, and then a linear interpolation allowed to find the function $$f(\text{sensor reading}) = \text{distance} = \alpha \cdot \text{sensor\_reading} + \beta$$ that maps sensor readings to obstacle distance.  
    The mapping has been calculated to be: $distance = n \cdot sensor\_reading + m$

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
