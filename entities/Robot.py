'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

import numpy as np
import math

from utils import util

class Robot():
    
    # Robot length(L) and radius(R)
    L = 0.075
    R = 0.02

    def __init__(
        self,
        control: object,
        env: str = 'simulation',
        robot_id: int = 0,
        team_color: bool = True # True: blue_team | False: yellow_team
    ) -> None:
        
        self.robot_id = robot_id
        self.team_color = team_color
        
        self.control = control
        self.env = env
        
        self.position = [.0,.0]
        self.orientation = .0
        
        self.wl, self.wr = 0, 0
        self.desired_speed = [.0,.0]

    def team_color_str(self) -> str:
        if self.team_color:
            return 'blue'
        else:
            return 'yellow'
    
    def set_desired(self, desired_speed):
        self.desired_speed = desired_speed

    def update(self, frame) -> None:
        """
        Updates the robot's own state
        """
        if self.team_color:
            _team_color = 'robotsBlue'
        else:
            _team_color = 'robotsYellow'

        if self.env == 'real':
            frame = frame.get('detection')
        
        for robot in frame[_team_color]:
            if robot.get('robotId') == self.robot_id:
                self.position = [
                    robot.get('x', 0),
                    robot.get('y', 0),
                ]
                self.orientation = robot.get('orientation', 0)
            
        self.control.update(self)
        
    def printInfo(self):
        print(' ')
        print('POS: ' + self.position)
        print('THETA: ' + self.orientation)
        print(' ')