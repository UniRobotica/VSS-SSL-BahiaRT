'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

from collections import deque
import math
import numpy as np

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
        self.front_position = [.0,.0]
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
        
    def _update_front_position(self):
        """
        Calculates and updates the robot's frontal position based on its rotation reference frame
        """
        r_front_angle = util.wrap_to_pi(self.orientation)
        to_sum = np.array([
            math.cos(r_front_angle),
            math.sin(r_front_angle)
        ])

        r_center_pos = np.array(self.position)

        r_front_pos = np.add(
            r_center_pos,
            np.multiply(to_sum, self.L/2)
        )

        self.front_position = r_front_pos
    
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
                    self._update_front_position()
                    
    def calculate_local_speeds(self, vg: list[float]):
        
        #proporcional angular
        n = (1/0.1)
        
        theta = util.apply_angular_decay(self.orientation, 1)
        
        v = vg[0] * math.cos(-theta) - vg[1] * math.sin(-theta)
        w = n * (vg[0] * math.sin(-theta) + vg[1] * math.cos(-theta))
        
        #inversed front side
        # desired_angle_rad = math.atan2(vg[1], vg[0])
        # desired_angle_deg = util.convert_to_deg(desired_angle_rad)

        # robot_angle_deg = util.convert_to_deg(orientation)
        # robot_angle_deg_inverted = util.add_deg(robot_angle_deg, 180)
        
        # angle_error_1 = desired_angle_deg - robot_angle_deg
        # angle_error_2 = desired_angle_deg - robot_angle_deg_inverted
        
        # if abs(angle_error_2) < abs(angle_error_1):
        #    w *= -1
        
        return v, w

    def speed_to_ws(self, v: float, w: float):
        """
        Returns the wheels angular speed

        Args:
            v (float): Linear speed
            w (float): Angular speed

        Returns:
            [wl, wr](float): wl - Left wheel angular speed, wr - Right wheel angular speed
        """
        wl = (2 * v - w * self.L)/2 * self.R
        wr = (2 * v + w * self.L)/2 * self.R
        
        return wl, wr

    def global_to_ws(self, vg: list[float]):
        """
        Gives the wheels power to reach the configuration [vx, vy, omega].

        Args:
            vg (list[float]): Global speed [vx, vy]
            orientation (float): Global orientation [omega]

        Returns:
            list(float): Wheels power [wl, wr]
        """
        
        v, w = self.calculate_local_speeds(vg)
        return np.array(self.speed_to_ws(v, w))
                    
        
    def print_info(self):
        """
        Prints robot information.
        """
        print(f'ROBOT: {self.robot_id} TEAM: {self.team_color_str()}')
        print(f'position: [{self.position[0]:.3f}, {self.position[1]:.3f}]')
        print(f'front position: [{self.front_position[0]:.3f}, {self.front_position[1]:.3f}]')
        print(f'theta: {np.rad2deg(self.orientation):.3f}')
        print(f'wl: {self.wl:.3f}')
        print(f'wr: {self.wr:.3f}')
        print(' ')