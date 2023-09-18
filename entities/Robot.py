'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

from collections import deque
import math

from utils import util
from utils.speed import speed, angular_speed

class Robot():
    
    # Robot length
    L = 0.075
    # Robot wheel radius
    R = 0.035

    def __init__(
        self,
        env: str = 'simulation',
        robot_id: int = 0,
        team_color: bool = True # True: blue_team | False: yellow_team
    ) -> None:
        
        self.robot_id = robot_id
        self.team_color = team_color
        
        self.env = env
        
        self.position = [.0,.0]
        self.orientation = .0
        
        self.wl, self.wr = .0, .0
        
        self.vx, self.vy, self.vtheta = .0, .0, .0
        self.speed = .0
        
        self.frames_info = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
            'theta': deque(maxlen=10),
            'fps': float
        }

    def team_color_str(self) -> str:
        if self.team_color:
            return 'blue'
        else:
            return 'yellow'
    
    def set_desired(self, power_w):
        self.wl = power_w[0]
        self.wr = power_w[1]

    def _update_speeds(self):
        self.frames_info['x'].append(self.position[0])
        self.frames_info['y'].append(self.position[1])
        self.frames_info['theta'].append(self.orientation)

        self.vx = speed(self.frames_info['x'], self.frames_info['fps'])
        self.vy = speed(self.frames_info['y'], self.frames_info['fps'])
        self.vtheta = angular_speed(self.frames_info['theta'], self.frames_info['fps'])

        self.speed = math.sqrt(self.vx ** 2 + self.vy ** 2)
    
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

        fps = frame.get('fps', None)
        if fps: 
            self.frames_info['fps'] = fps
        
        frame = frame.get(_team_color) if frame.get(_team_color) else None
        if frame:
            for robot in frame:
                if robot.get('robotId', 0) == self.robot_id:
                    self.position = [
                        robot.get('x', 0),
                        robot.get('y', 0),
                    ]
                    self.orientation = robot.get('orientation', 0)
                    
        
    def printInfo(self):
        print(' ')
        print('POS: ' + self.position)
        print('THETA: ' + self.orientation)
        print(' ')