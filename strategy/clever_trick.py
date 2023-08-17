from entities.Robot import Robot
import math
from utils import util

class CleverTrick():

    def __init__(
        self,
        consider_back=True,
        force=1
    ) -> None:
        
        self.consider_back = consider_back
        self.force = force
        self.desired_speed = [.0, .0]
        
    def control(self, robot: Robot):
        
        #proporcional angular
        n = (1/0.185)
        
        theta = util.apply_angular_decay(robot.orientation, 1) 
        
        v = self.desired_speed[0] * math.cos(-theta) - self.desired_speed[1] * math.sin(-theta)
        w = n * (self.desired_speed[0] * math.sin(-theta) + self.desired_speed[1] * math.cos(-theta))
        
        if self.consider_back:
            
            desired_angle_rad = math.atan2(self.desired_speed[1], self.desired_speed[0])
            desired_angle_deg = util.convertTodeg(desired_angle_rad)
            
            robot_angle_rad = robot.orientation
            robot_angle_deg = util.convertTodeg(robot_angle_rad)
            robot_angle_deg_inverted = util.add_deg(robot_angle_deg, 180)
            
            angle_error_1 = desired_angle_deg - robot_angle_deg
            angle_error_2 = desired_angle_deg - robot_angle_deg_inverted
            
            if abs(angle_error_2) < abs(angle_error_1):
                w *= -1

        wl = (2 * v - w * robot.L)/2 * robot.R
        wr = (2 * v + w * robot.L)/2 * robot.R
        
        return wl, wr
    
    def update(self, robot: Robot):
        
        self.desired_speed = robot.desired_speed
        wl, wr = self.control(robot)
        
        return wl * self.force, wr * self.force