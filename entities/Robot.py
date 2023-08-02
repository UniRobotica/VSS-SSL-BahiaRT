'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

import numpy as np
import math

def apply_angular_decay(angular_velocity: float, decay_rate: float) -> float:
    return angular_velocity * decay_rate

def detect_ball_proximity(ball_position: list[float], robot_position: list[float]) -> float:
    distance = np.linalg.norm(np.array(ball_position) - np.array(robot_position))
    return distance

class Robot():
    '''
    A base class that contains the state of the robot and basic movement methods for all game positions.
    '''
    # Robot length(L) and radius(R)
    L = 0.075
    R = 0.02

    def __init__(
        self,
        position: list[float] = [],
        velocity: list[float] = [],
        robot_id: int = 0,
        team_color: bool = True # True: blue_team | False: yellow_team
    ) -> None:
        self.robot_id = robot_id
        self.team_color = team_color
        self.position = position
        self.velocity = velocity

    def team_color_str(self) -> str:
        if self.team_color:
            return 'blue'
        else:
            return 'yellow'

    def update(self, frame) -> None:
        """
        Updates the robot's own state
        """
        if self.team_color:
            _team_color = 'robotsBlue'
        else:
            _team_color = 'robotsYellow'

        robots = frame.get('detection').get(_team_color)
        for robot in robots:
            if robot.get('robotId') == self.robot_id:
                self.position = [
                    robot.get('x', 0),
                    robot.get('y', 0),
                    robot.get('orientation', 0)
                ]
        
    def printInfo(self):
        print('-----------------------------')
        print(self.position)
        print(self.velocity)
        print('-----------------------------')
        
    def clever_trick(self, vector_speed: list[float]) -> list[float]:
        """
        Gives the wheels angular speed based on a speed vector using the clever trick expression.

        Args:
            vector_speed (Vector): An array [dx, dy]

        Returns:
            wl, wr: Floats, where wl is the left wheel angular velocity and wr is right wheel angular velocity.
        """
        
        #proporcional angular
        n = (1/0.185)
        
        theta = apply_angular_decay(self.position.theta, 1)
        v = vector_speed[0] * math.cos(-theta) - vector_speed[1] * math.sin(-theta)
        w = n * (vector_speed[0] * math.sin(-theta) + vector_speed[1] * math.cos(-theta))

        wl = (2 * v - w * self.L)/2 * self.R
        wr = (2 * v + w * self.L)/2 * self.R

        return wl, wr