'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

from entities.Robot import Robot
from geometry.Point import Point
from geometry.Pose import Pose
from geometry.Vector import Vector
from game.Game import Game
import math
import numpy as np
 
def apply_angular_decay(angular_velocity: float, decay_rate: float) -> float:
    return angular_velocity * decay_rate

def detect_ball_proximity(ball_position: list[float], robot_position: list[float]) -> float:
    distance = np.linalg.norm(np.array(ball_position) - np.array(robot_position))
    return distance

class Player(Robot):
    
    def __init__(
        self, 
        game: Game,
        position: Pose = Pose(),
        velocity: Vector = Vector(),
        robot_id: int = 0,
        team_color: bool = True
    ) -> None:
        super().__init__(game, position, velocity, robot_id, team_color)
    
    def clever_trick(self, vector_speed: Vector) -> list[float]:
        """
        Gives the wheels angular speed based on a speed vector using the clever trick expression.

        Args:
            vector_speed (Vector): An array [dx, dy]

        Returns:
            list[float]: An array [wl, wr] where wl is the left wheel angular velocity and wr is right wheel angular velocity.
        """
        
        #proporcional angular
        n = (1/0.185)
        
        theta = apply_angular_decay(self.position.theta, 1)
        v = vector_speed.x * math.cos(-theta) - vector_speed.y * math.sin(-theta)
        w = n * (vector_speed.x * math.sin(-theta) + vector_speed.y * math.cos(-theta))

        wl = (2 * v - w * self.L)/2 * self.R
        wr = (2 * v + w * self.L)/2 * self.R

        return [wl, wr]
    
    def move_ball_to(self, position: Point) -> None:
        pass

    def sweep_ball(self):
        pass

    def position_to_sweep_ball(self):
        pass