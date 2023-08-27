import numpy as np
import math

from utils import util
from entities.Robot import Robot

CONSTANTS = {
    'simulation': {
        'D_E': 0.05,
        'K_P': 0,
        'K_O': 0.0012,
        'D_MIN': 0.0348,
        'DELTA' : 0.0457
    },
    'real': {
        'D_E': 0.1537,
        'K_P': 0.0415,
        'K_O': 0.0012,
        'D_MIN': 0.0348,
        'DELTA' : 0.0457
    }
}


def Nh(phi: float) -> list[float]:
    
    return np.array([math.cos(phi), math.sin(phi)])


class HyperbolicField():
    '''
    Um campo potencial hiperbólico que pode ter uma força de sentido horário ou anti-horário.
    '''    
    
    def __init__(
        self, 
        home_point: list[float], 
        cw: bool,
        env: str
    ) -> None:
      
        self.home_point = home_point
        self.env = env
        self.direction = -1 if cw else 1
        self.radius = CONSTANTS[self.env]['D_E']
        self.kp = CONSTANTS[self.env]['K_P']
      
    def update_home_point(self, home_pos):
      
        self.home_point = home_pos
    
    def compute(self, object_position: list[float]) -> list[float]:
        
        theta = util.angleBetweenTwoPoints(self.home_point, object_position)
        p = util.distanceBetweenTwoPoints(self.home_point, object_position)
        
        if p > self.radius:
            phi = theta + self.direction * (math.pi/2 * (2 - (self.radius + self.kp)/(p + self.kp)))
            
        elif p <= self.radius and p >= 0:
            phi = theta + self.direction * (math.pi/2 * (p/self.radius)**0.5)
            
        return phi


class RepulsivePointField():
    '''
    Um campo potencial de atração originário de um único ponto.
    '''
    
    def __init__(
        self, 
        home_pos: list[float],
        radius: float
    ) -> None:
        self.home_pos = home_pos
        self.radius = radius
    
    def update(self, home_pos):
      
        self.home_pos = home_pos
    
    def compute(self, object_position: list[float]) -> list[float]:
        
        p = util.distanceBetweenTwoPoints(self.home_pos, object_position)
        
        if p <= self.radius:
            
            return util.angleBetweenTwoPoints(self.home_pos, object_position)
        
        else:
            
            return None
        

class MoveToGoalField():
    
    def __init__(
        self,
        home_point: list[float],
        env
    ) -> None:
    
        self.home_point = home_point
        self.env = env
        self.radius = CONSTANTS[self.env]['D_E']
        
        self.h_field_ccw = HyperbolicField(
            self.home_point,
            False,
            self.env
        )
        self.h_field_cw = HyperbolicField(
            self.home_point,
            True,
            self.env
        )
        
    def update_home_point(self, home_point):
        
        self.home_point = home_point
        
    def compute(self, object_position: list[float]):
        
        yl = abs(object_position[1] + self.radius)
        yr = abs(object_position[1] - self.radius)
        
        dy = abs(object_position[1] - self.home_point[1])
        
        phi_ccw = self.h_field_ccw.compute(
            [object_position[1], object_position[1] - self.radius]
        )
        phi_cw = self.h_field_cw.compute(
            [object_position[1], object_position[1] + self.radius]
        )
        
        if -self.radius <= dy < self.radius:
            
            spiral_merge = (yl * Nh(phi_ccw) + yr * Nh(phi_cw)) / 2 * self.radius
            return math.atan2(spiral_merge[1], spiral_merge[0])
        
        elif dy < -self.radius:
            
            return phi_cw
        
        elif dy >= self.radius:
            
            return phi_ccw