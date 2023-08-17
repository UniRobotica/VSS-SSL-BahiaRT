'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import math
import numpy as np
from types import FunctionType

from utils import util
from entities.Robot import Robot

'''
K_P : Constante de ajuste
'''
CONSTANTS = {
    'simulation': {
        'K_P': 0
    },
    'real_life': {
        'K_P': 0
    }
}

class AttractivePointField():
    """
    Represents a potential field that has a point as the origin
    """
    
    def __init__(
        self,
        home_pos: list[float], 
        decay: FunctionType = lambda x:1, 
        max_radius: float = 1.0,
    ) -> None:
        
        self.home_pos = home_pos
        self.decay = decay
        self.max_radius = max_radius
        self.type = type
    
    def update(self, home_pos):
      
        self.home_pos = home_pos
        
    def getForce(self, target_position, force_multiply=1.0) -> list[float]:
        
        if self.type:

            to_target = [
                self.home_pos[0] - target_position[0], self.home_pos[1] - target_position[1]
            ]

            to_target_scalar = np.linalg.norm(to_target)
            to_target_scalar_norm = max(0, min(1, to_target_scalar/self.max_radius))
            
            force = self.decay(to_target_scalar_norm)
            to_target_norm = util.unit_vector(to_target)
            
            return [
                    to_target_norm[0] * force * force_multiply,
                    to_target_norm[1] * force * force_multiply
            ]
        
        else:
            # Not implemented
            pass

class HyperbolicField():
    
    def __init__(
        self, 
        home_pos: list[float], 
        radius: float,
        direction: bool = True,
    ) -> None:
      
        self.home_pos = home_pos
        self.radius = radius
        self.kp = CONSTANTS['simulation']['K_P']
        self.direction = 1 if direction else -1
      
    def update(self, home_pos):
      
        self.home_pos = home_pos
      
    def getForce(self, object_pos: list[float], force_multiply: float=1.0) -> list[float]:
        
        theta = util.angleBetweenTwoPoints(self.home_pos, object_pos)
        p = (object_pos[1] - self.home_pos[1])**2 + (object_pos[0] - self.home_pos[1])**2
        
        if p > self.radius:
            theta_d = theta + self.direction * (math.pi/2 * (2 - (self.radius + self.kp)/(p + self.kp)))
            
        if p <= self.radius and p >= 0:
            theta_d = theta + self.direction * (math.pi/2 * (p/self.radius)**0.5)
        
        return math.cos(theta_d) * force_multiply, math.sin(theta_d) * force_multiply
    
class RepulsivePointField():
    
    def __init__(
        self, 
        home_pos: list[float],
        range: float = 0
    ) -> None:
        self.home_pos = home_pos
        self.range = range
    
    def update(self, home_pos):
      
        self.home_pos = home_pos
    
    def getForce(self, object_pos: list[float], force_multiply: float=1.0) -> list[float]:
        
        theta = util.angleBetweenTwoPoints(self.home_pos, object_pos)
        
        if self.range == 0:
            return math.cos(theta) * force_multiply, math.sin(theta) * force_multiply
        else:
            p = (object_pos[1] - self.home_pos[1])**2 + (object_pos[0] - self.home_pos[1])**2
            
            if p <= self.range:
                return math.cos(theta) * force_multiply, math.sin(theta) * force_multiply
            else:
                return 0,0

class MoveToGoalField():
    pass