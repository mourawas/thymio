import math
import numpy as np
class ThymioControl:
    def __init__(self):
        # Thymio's pose and old pose
        self.__pos = []
        self.__angle = None      
        self.__oldPos = []
        self.__oldAngle = None

        # value of the timestep considered in the control loop
        # used for the derivative part of the controller
        self.__timeStep = 0

        # previous error for the derivative part of the controller
        self.__prevAngleError = 0

        # index of the current waypoint in the path that is being followed
        self.__step = 1

        # threshold for the kidnapping detection
        self.__kidnappingThresholdPosition = 20 # mm
        self.__kidnappingThresholdAngle = 30 # degrees

        # threshold for the robot to consider that it has reached a waypoint
        self.__reachedThreshold = 20 # mm

        # threshold for the robot to consider that it has reached a desired orientation
        self.__angleThreshold = 0.1 # rad

        # constant linear speed
        self.__linearSpeed = 50 # mm/s

        # conversion from Thymio wheel speed commands to mm/s
        self.__thymioWheelSpeedConversion = 0.3726 # (mm/s)/pwm

        # constant proportional parameter for transforming angle into rotational speed
        self.__proportionalGain = 0.7

        # constant derivative parameter for transforming angle into rotational speed
        self.__derivativeGain = 0.5

        # adjustment for the thymio's wheels differences
        self.__wheelsAdjustment = 1.07

        # cell to mm conversion
        self.__cellToMm = 100 # 1 cell = self.__cellToMm mm

        # max value for angular speed when rotating
        self.__maxAngularSpeed = 2 * math.pi
        
        # robot geometry
        self.__lenght = 93 # mm, distance between the wheels

    # timestep setter
    def set_timestep(self, timestep):
        self.__timeStep = timestep

    # pose setter
    def set_pose(self, position, angle):
        self.__pos = [position[0] * self.__cellToMm, position[1] * self.__cellToMm]
        self.__angle = angle

    # pose getter
    def get_position(self):
        return self.__pos

    # path setter
    def set_path(self, path):
        self.__path = []
        for position in path:
            position = np.array(position)
            self.__path.append((float(position[1] * self.__cellToMm), float(position[0] * self.__cellToMm)))
        # remove the cells that are in a straight line
        self.__reduce_path()
        self.__step = 1

    # reduce the path by removing the cells that are in a straight line
    def __reduce_path(self):
        i = 0
        while i < len(self.__path) - 2:
            x1, y1 = self.__path[i]
            x2, y2 = self.__path[i + 1]
            x3, y3 = self.__path[i + 2]
            if (x1 - x2) * (y2 - y3) == (y1 - y2) * (x2 - x3):
                self.__path.pop(i + 1)
            else:
                i += 1
    
    # path getter, in cell format
    def get_path_cells(self):
        new_path = []
        for position in self.__path:
            position = np.array(position)
            new_path.append((float(position[1] / self.__cellToMm), float(position[0] / self.__cellToMm)))
        return new_path
    
    # update the pose of the robot, saving the old pose
    def update_pose(self, position, angle):
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
        self.__pos = [position[0] * self.__cellToMm, position[1] * self.__cellToMm]
        self.__angle = angle
    
    # check if the robot is kidnapped
    def amIKidnapped(self):
        return math.sqrt((self.__oldPos[0] - self.__pos[0])**2 + (self.__oldPos[1] - self.__pos[1])**2) > self.__kidnappingThresholdPosition or abs(self.__oldAngle - self.__angle) > self.__kidnappingThresholdAngle
    
    # move the robot along the path, using PD controller and costant linear speed
    def move_pd(self, position, angle):
        goal = False

        #position and angle of the Thymio
        self.__angle = angle
        self.__pos = position

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        x_diff = objective[0] - self.__pos[0]
        y_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(x_diff**2 + y_diff**2)
        print("current objective: ", self.__step)
        print("objective: ", objective, " position: ", self.__pos)
        print("x_diff: ", x_diff, " y_diff: ", y_diff, " distance: ", distance)

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        print("my angle: ", self.__angle)
        print("waypoint angle: ", math.atan2(y_diff, x_diff))
        angleDistance = (math.atan2(y_diff, x_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi
        print("angleDistance: ", angleDistance)

        # move the robot and if the cell is reached, delete it and restart with the following
        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1
            print("Next objective: ", self.__step)  

            v, w, wl, wr, goal = self.move(position, angle)
        else:
            # move throwards the next cell in the path
            # constant linear speed, PD controller for the angular speed
            v, w = self.__linearSpeed, angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            self.__prevAngleError = angleDistance
            w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
            wl, wr = self.differentialDrive(v, w)
        return v, w, wl, wr, goal
    
    # move the robot along the path, stopping and turning with PD controller before going straight to the waypoints
    def move(self, position, angle):

        #position and angle of the thymio
        self.__angle = angle
        self.__pos = position

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        x_diff = objective[0] - self.__pos[0]
        y_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(x_diff**2 + y_diff**2)
        print("current objective: ", self.__step)
        print("objective: ", objective, " position: ", self.__pos)
        print("x_diff: ", x_diff, " y_diff: ", y_diff, " distance: ", distance)

        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1
            print("Next objective: ", self.__step)
            
            objective = self.__path[self.__step]

            # calculate the new distance between the robot and the objective
            x_diff = objective[0] - self.__pos[0]
            y_diff = objective[1] - self.__pos[1]
            distance = math.sqrt(x_diff**2 + y_diff**2)
            print("x_diff: ", x_diff, " y_diff: ", y_diff, " distance: ", distance)

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        print("my angle: ", self.__angle)
        print("waypoint angle: ", math.atan2(y_diff, x_diff))
        angleDistance = (math.atan2(y_diff, x_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi
        print("angleDistance: ", angleDistance)

        if angleDistance > self.__angleThreshold or angleDistance < -self.__angleThreshold:
            # if the angle is not the desired one, rotate in place
            w = angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            v = 0
        else:
            # if the angle is the desired one, move straight
            w = 0
            v = self.__linearSpeed

        # clip the angular speed
        w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
        wl, wr = self.differentialDrive(v, w)

        return v, w, wl, wr, False
    
    # differential drive model, using Thymio's geometry
    def differentialDrive(self, v, w):
        
        # notice the inverted sign, because the map has the y axis inverted
        vr = (v - w * self.__lenght / 2)
        vl = (v + w * self.__lenght / 2)

        wr = vr / self.__thymioWheelSpeedConversion
        wl = vl * self.__wheelsAdjustment / self.__thymioWheelSpeedConversion

        return int(wl), int(wr)
    
    # inverse differential drive model, using Thymio's geometry
    def inverseDifferentialDrive(self, wl, wr):
        vr = wr * self.__thymioWheelSpeedConversion # mm/s from pwd
        vl = wl * self.__thymioWheelSpeedConversion / self.__wheelsAdjustment # mm/s from pwd

        v = (vr + vl) / 2
        w = (vl - vr) / self.__lenght

        return v, w
    
    # convert the speed from mm/s to cells/s
    def convert_speed_cells(self, speed):
        return speed / self.__cellToMm
    
    # return the distance between the wheels
    def get_wheel_distance(self):
        return self.__lenght
    
    # convert the distance from mm to cells
    def mm_to_cells(self, mm):
        return mm / self.__cellToMm
