import numpy as np
import math

from utils import util
from entities.Robot import Robot

CONSTANTS = {
    'simulation': {
        'D_E': 0.1,
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
        self.cw = not cw
        self.radius = CONSTANTS[self.env]['D_E']
        self.kp = CONSTANTS[self.env]['K_P']
      
    def update_home_point(self, home_pos):
      
        self.home_point = home_pos
    
    def compute(self, dx: float, dy: float) -> float:
        
        theta = math.atan2(dy, dx)
        rho = util.norm(dx, dy)
        
        if rho > self.radius:
            angle = (math.pi / 2) * (2 - ((self.radius + self.kp) / (rho + self.kp)))
        elif 0 <= rho <= self.radius:
            angle = (math.pi / 2) * math.sqrt(rho / self.radius)

        if self.cw:
            return util.wrapToPi(theta + angle)
        else:
            return util.wrapToPi(theta - angle)
        

class RepulsivePointField():
    '''
    Um campo potencial de atração originário de um único ponto.
    '''
    
    def __init__(
        self, 
        home_point: list[float],
        env: str
    ) -> None:
        self.home_point = home_point
        self.env = env
    
    def update(self, home_pos):
      
        self.home_pos = home_pos
    
    def compute(self, dx, dy: list[float]) -> list[float]:
        
        return math.atan2(dy, dx)
        

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
        
    def compute(self, dx: float, dy: float) -> float:
        
        yl = dy + self.radius
        yr = dy - self.radius

        phi_ccw = self.h_field_cw.compute(dx, dy - self.radius)
        phi_cw = self.h_field_ccw.compute(dx, dy + self.radius)

        nh_ccw = Nh(phi_ccw)
        nh_cw = Nh(phi_cw)
        # The absolute value of y_l and y_r was not specified in the article, but the obtained results 
        # with this trick are closer to the article images
        spiral_merge = (abs(yl) * nh_ccw + abs(yr) * nh_cw) / (2 * self.radius) 

        if dy > self.radius:
            phi_tuf = self.h_field_cw.compute(dx, dy - self.radius)
        elif dy < -self.radius:
            phi_tuf = self.h_field_ccw.compute(dx, dy + self.radius)
        else:
            phi_tuf = math.atan2(spiral_merge[1], spiral_merge[0])
            
        return util.wrapToPi(phi_tuf)
    

class AvoidObstacleField():
    
    def __init__(
        self,
        home_point: list[float]
    ) -> None:
        
        self.home_point = home_point
        