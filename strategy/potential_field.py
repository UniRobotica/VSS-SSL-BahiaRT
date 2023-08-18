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
    '''
    Um campo potencial de atração originário de um único ponto.
    '''
    
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
    '''
    Um campo potencial hiperbólico que pode ter uma força de sentido horário ou anti-horário.
    '''    
    
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
        p = util.distanceBetweenTwoPoints(self.home_pos, object_pos)
        
        if p > self.radius:
            theta_d = theta + self.direction * (math.pi/2 * (2 - (self.radius + self.kp)/(p + self.kp)))
            
        if p <= self.radius and p >= 0:
            theta_d = theta + self.direction * (math.pi/2 * (p/self.radius)**0.5)
        
        return math.cos(theta_d) * force_multiply, math.sin(theta_d) * force_multiply
    
class RepulsivePointField():
    '''
    Um campo potencial de atração originário de um único ponto.
    '''
    
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
            p = util.distanceBetweenTwoPoints(self.home_pos, object_pos)
            
            if p <= self.range:
                return math.cos(theta) * force_multiply, math.sin(theta) * force_multiply
            else:
                return 0,0

class MoveToGoalField():
    '''
    Um campo potencial de atração originário de um único ponto, que objetiva fazer com que o robô chegue ao objeto com a orientação correta.
    '''
    
    def __init__(
        self, 
        home_pos: list[float],
        radius: float,
    ) -> None:
        
        self.home_pos = home_pos
        self.radius = radius
        self.hyperbolic_field_cw = HyperbolicField(
            home_pos=self.home_pos,
            radius=self.radius,
            direction=1
        )
        self.hyperbolic_field_ccw = HyperbolicField(
            home_pos=self.home_pos,
            radius=self.radius,
            direction=-1
        )
        
    def update(self, home_pos):
      
        self.home_pos = home_pos
    
    def getForce(self, object_pos: list[float], force_multiply: float=1.0) -> list[float]:
        
        yl = object_pos[1] + self.radius
        yr = object_pos - self.radius
        
        if object_pos[1] < self.radius and object_pos[1] >= -self.radius:
            
            v_theta_ccw = self.hyperbolic_field_ccw.getForce([object_pos[0], yl])
            v_theta_cw = self.hyperbolic_field_cw.getForce([object_pos[0], yr])
            
            x = (yl * v_theta_ccw + yr * v_theta_cw) / 2 * self.radius
        
            #to implement
            
        if object_pos[1] < -self.radius:
            
            self.hyperbolic_field_cw.update(self.home_pos)
            return self.hyperbolic_field_ccw.getForce([object_pos[0], yr])
            
        if object_pos[1] >= self.radius:
            
            self.hyperbolic_field_ccw.update(self.home_pos)
            return self.hyperbolic_field_ccw.getForce([object_pos[0], yl])