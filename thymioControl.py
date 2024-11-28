import math
class ThymioControl:
    def __init__(self):
        self.__pos = []        
        self.__oldPos = []
        self.__angle = None
        self.__oldAngle = None

        self.__timeStep = 0

        self.__prevAngleError = 0

        self.__step = 1

        # thresholds
        self.__kidnappingThresholdPosition = 20
        self.__kidnappingThresholdAngle = 30 # degrees
        self.__reachedThreshold = 20 # mm
        self.__angleThreshold = 0.1 # rad

        # constant linear speed
        self.__linearSpeed = 500

        # conversion from rad/s to wheel speed command
        # self.__thymioWheelSpeedConversion = 65.5
        self.__thymioWheelSpeedConversion = 0.43 # (mm/s)/pwd
        # constant proportional parameter for transforming angle into rotational speed
        self.__proportionalGain = 0.7
        # constant derivative parameter for transforming angle into rotational speed
        self.__derivativeGain = 0.5
        # adjustment for the thymio's wheels
        self.__wheelsAdjustment = 1.1
        # cell to mm conversion
        self.__cellToMm = 100

        # max value for angular speed
        self.__maxAngularSpeed = 2 * math.pi
        
        # robot geometry
        self.__lenght = 93 # mm, distance between the wheels
        self.__radius = 22 # mm, radius of the wheels

        # kalman filter state
        self.x_cam = 0
        self.y_cam = 0
        self.theta_cam = 0
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0

    def set_timestep(self, timestep):
        self.__timeStep = timestep

    def set_pose(self, position, angle):
        print("Setting pose, position: ", position, " angle: ", angle)
        self.__pos = [position[0] * self.__cellToMm, position[1] * self.__cellToMm]
        self.__angle = angle

    def get_position(self):
        return self.__pos

    def set_path(self, path):
        self.__path = []
        for position in path:
            self.__path.append([position[1] * self.__cellToMm, position[0] * self.__cellToMm])
        self.__reduce_path()
        self.__step = 1

    def __reduce_path(self):
        # reduce the path by removing the cells that are in a straight line
        i = 0
        while i < len(self.__path) - 2:
            x1, y1 = self.__path[i]
            x2, y2 = self.__path[i + 1]
            x3, y3 = self.__path[i + 2]
            if (x1 - x2) * (y2 - y3) == (y1 - y2) * (x2 - x3):
                self.__path.pop(i + 1)
            else:
                i += 1
    
    def get_path_cells(self):
        new_path = []
        for position in self.__path:
            new_path.append([position[1] / self.__cellToMm, position[0] / self.__cellToMm])
        return new_path
    
    def update_pose(self, position, angle):
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
        self.__pos = [position[0] * self.__cellToMm, position[1] * self.__cellToMm]
        self.__angle = angle
    
    def amIKidnapped(self):
        # check if the robot is kidnapped
        return math.sqrt(math.pow(self.__oldPos[0] - self.__pos[0]) + math.pow(self.__oldPos[1] - self.__pos[1])) > self.__kidnappingThresholdPosition or abs(self.__oldAngle - self.__angle) > self.__kidnappingThresholdAngle
    
    def move_pd(self, position, angle):
        goal = False
        # move the robot along the path

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

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        print("my angle: ", self.__angle)
        print("waypoint angle: ", math.atan2(y_diff, x_diff))
        print("diff angle: ", math.atan2(y_diff, x_diff) - self.__angle)
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
            v, w = self.__linearSpeed, angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            self.__prevAngleError = angleDistance
            w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
            wl, wr = self.differentialDrive(v, w)

            # find wl and wr with the astolfi controller
        return v, w, wl, wr, goal
    
    def move(self, position, angle):
        # move the robot along the path

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
        print("diff angle: ", math.atan2(y_diff, x_diff) - self.__angle)
        angleDistance = (math.atan2(y_diff, x_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi
        print("angleDistance: ", angleDistance)

        if angleDistance > self.__angleThreshold or angleDistance < -self.__angleThreshold:
            w = angleDistance * self.__proportionalGain + self.__derivativeGain * (angleDistance - self.__prevAngleError) / self.__timeStep
            v = 0
        else:
            w = 0
            v = self.__linearSpeed

        w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
        wl, wr = self.differentialDrive(v, w)

        return v, w, wl, wr, False
    
    def differentialDrive(self, v, w):
        wl = (1 / self.__thymioWheelSpeedConversion) * self.__wheelsAdjustment * (v - self.__lenght * w / 2) / self.__radius
        wr = (1 / self.__thymioWheelSpeedConversion) * (v + self.__lenght * w / 2) / self.__radius
        return wl, wr
    
    def inverseDifferentialDrive(self, wl, wr):
        w = ((wr - wl/self.__wheelsAdjustment) * self.__radius * self.__thymioWheelSpeedConversion) / self.__lenght
        v = ((wr + wl/self.__wheelsAdjustment) * self.__radius * self.__thymioWheelSpeedConversion) / 2
        return v, w
    
    def convert_speed_cells(self, speed):
        return speed / self.__cellToMm
    
    def get_wheel_distance(self):
        return self.__lenght
    
    def mm_to_cells(self, mm):
        return mm / self.__cellToMm
