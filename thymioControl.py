import math
import numpy as np
class ThymioControl:
    def __init__(self):
        # Thymio's pose and old pose
        self.__pos = []
        self.__angle = None      
        self.__oldPos = []
        self.__oldAngle = None

        # Thymio's predicted pose, used for kidnapping detection
        self.__pos_est = []
        self.__angle_est = None

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
        self.__reachedThreshold = 50 # mm

        # threshold for the robot to consider that it has reached a desired orientation
        self.__angleThreshold = 0.1 # rad

        # constant linear speed
        self.__linearSpeed = 100 # mm/s

        # conversion from Thymio wheel speed commands to mm/s
        self.__thymioWheelSpeedConversion = 0.3726 # (mm/s)/pwm

        # constant proportional parameter for transforming angle into rotational speed
        self.__proportionalGain = 0.3

        # constant derivative parameter for transforming angle into rotational speed
        self.__derivativeGain = 0.3

        # adjustment for the thymio's wheels differences
        self.__wheelsAdjustment = 1.07

        # cell to mm conversion
        self.__cellToMm = 0 # 1 cell = self.__cellToMm mm

        # max value for angular speed when rotating
        self.__maxAngularSpeed = 2 * math.pi
        
        # robot geometry
        self.__lenght = 93.5 # mm, distance between the wheels

    def set_scale(self, scale):
        # scale is cells per mm, I want mm per cell
        self.__cellToMm = 1 / scale

    # pose setter
    def set_pose(self, position, angle):
        if position is not None:
            self.__pos = [float(position[0] * self.__cellToMm), float(position[1] * self.__cellToMm)]
            self.__angle = angle

        print("THYMIO: pos: ", self.__pos)
        print("THYMIO: angle: ", self.__angle)

    # pose getter
    def get_position(self):
        return self.__pos
    
    # angle getter
    def get_angle(self):
        return self.__angle

    # path setter
    def set_path(self, path):
        self.__path = []
        for position in path:
            position = np.array(position)
            self.__path.append((float(position[0] * self.__cellToMm), float(position[1] * self.__cellToMm)))
        # remove the cells that are in a straight line
        self.__reduce_path()
        self.__step = 1

    # reduce the path by removing the cells that are in a straight line
    def __reduce_path(self):
        # tolerance for floating-point comparisons
        epsilon = 1e-6
        i = 0
        while i < len(self.__path) - 2:
            x1, y1 = self.__path[i]
            x2, y2 = self.__path[i + 1]
            x3, y3 = self.__path[i + 2]

            # calculate cross-product to check collinearity
            cross_product = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
            
            # check if the points are approximately collinear
            if abs(cross_product) < epsilon:
                self.__path.pop(i + 1)
            else:
                i += 1
        
        self.__path = [(round(x, 2), round(y, 2)) for x, y in self.__path]
    
    # path getter, in cell format
    def get_path_cells(self):
        new_path = []
        for position in self.__path:
            position = np.array(position)
            new_path.append((float(position[0] / self.__cellToMm), float(position[1] / self.__cellToMm)))

        new_path = [(round(x, 2), round(y, 2)) for x, y in new_path]
        return new_path

    # update the pose of the robot, saving the old pose
    def update_pose(self, position, angle):
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
        self.__pos = [position[0], position[1]]
        self.__angle = angle
        print("THYMIO CONTROL: old pos: ", self.__oldPos)
        print("THYMIO CONTROL: old angle: ", self.__oldAngle)
        print("THYMIO CONTROL: pos: ", self.__pos)
        print("THYMIO CONTROL: angle: ", self.__angle)
    
    # check if the robot is kidnapped
    def amIKidnapped(self, position, angle):
        print("THYMIO CONTROL: kidnapping predicted pos: ", self.__pos_est)
        print("THYMIO CONTROL: kidnapping predicted angle: ", self.__angle_est)
        print("THYMIO CONTROL: kidnapping pos: ", position)
        print("THYMIO CONTROL: kidnapping angle: ", angle)
        pos = [float(position[0] * self.__cellToMm), float(position[1] * self.__cellToMm)]
        return self.__pos_est != [] and self.__angle_est != None and (math.sqrt((self.__pos_est[0] - pos[0])**2 + (self.__pos_est[1] - pos[1])**2) > self.__kidnappingThresholdPosition or abs(self.__angle_est - angle) > self.__kidnappingThresholdAngle)
    
    def set_pred(self, row, col, angle):
        self.__pos_est = [row, col]
        self.__angle_est = angle
    
    # move the robot along the path, using PD controller and costant linear speed
    def move_pd(self, position, angle, dt):
        goal = False

        self.__timeStep = dt

        #position and angle of the Thymio
        self.__angle = angle
        self.__pos = position

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        x_diff = objective[0] - self.__pos[0]
        y_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(x_diff**2 + y_diff**2)
        print("THYMIO CONTROL: objective: ", objective)
        print("THYMIO CONTROL: pos: ", self.__pos)
        print("THYMIO CONTROL: distance: ", distance)

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        angleDistance = (math.atan2(y_diff, x_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi

        # move the robot and if the cell is reached, delete it and restart with the following
        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("THYMIO CONTROL: Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1
            print("THYMIO CONTROL: Next objective: ", self.__step)
            v, w, wl, wr, goal = self.move_pd(position, angle, dt)
        else:
            # move throwards the next cell in the path
            # constant linear speed, PD controller for the angular speed
            if self.__timeStep != 0:
                v, w = self.__linearSpeed, angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            else:
                v, w = 0, 0
            self.__prevAngleError = angleDistance
            w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
            wl, wr = self.differentialDrive(v, w)
        return v, w, wl, wr, goal
    
    # move the robot along the path, stopping and turning with PD controller before going straight to the waypoints
    def move(self, position, angle, dt):

        self.__timeStep = dt

        #position and angle of the thymio
        self.__angle = angle
        self.__pos = position

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        row_diff = objective[0] - self.__pos[0]
        col_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(row_diff**2 + col_diff**2)
        print("THYMIO CONTROL: objective: ", objective)
        print("THYMIO CONTROL: pos: ", self.__pos)
        print("THYMIO CONTROL: distance: ", distance)

        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("THYMIO CONTROL: Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1
            print("THYMIO CONTROL: next objective: ", self.__step)
            
            objective = self.__path[self.__step]

            # calculate the new distance between the robot and the objective
            row_diff = objective[0] - self.__pos[0]
            col_diff = objective[1] - self.__pos[1]
            distance = math.sqrt(row_diff**2 + col_diff**2)
            print("THYMIO CONTROL: new objective: ", objective)
            print("THYMIO CONTROL: pos: ", self.__pos)
            print("THYMIO CONTROL: new distance: ", distance)

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        angleDistance = (math.atan2(col_diff, row_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi
        print("THYMIO CONTROL: waypoint angle: ", math.atan2(col_diff, row_diff))
        print("THYMIO CONTROL: angle: ", self.__angle)
        print("THYMIO CONTROL: angleDistance: ", angleDistance)

        if angleDistance > self.__angleThreshold or angleDistance < -self.__angleThreshold:
            # if the angle is not the desired one, rotate in place
            if self.__timeStep != 0:
                w = angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            else:
                w = 0
            v = 0
        else:
            # if the angle is the desired one, move straight
            w = 0
            v = self.__linearSpeed
        
        print("THYMIO CONTROL: v: ", v)
        print("THYMIO CONTROL: w: ", w)

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
        return float(speed / self.__cellToMm)
    
    # return the distance between the wheels
    def get_wheel_distance(self):
        return self.__lenght
    
    # convert the distance from mm to cells
    def mm_to_cells(self, mm):
        return float(mm / self.__cellToMm)
    
    def cells_to_mm(self, cells):
        return float(cells * self.__cellToMm)
