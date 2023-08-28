import numpy as np
import math

from utils import util
from entities.Robot import Robot

CONSTANTS = {
    'simulation': {
        'D_E': 0.2,
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
    
    def compute(self, d_x: float, d_y: float) -> list[float]:
        
        theta = math.atan2(d_y, d_x)
        rho = util.norm(d_x, d_y)
        
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
        radius: float
    ) -> None:
        self.home_point = home_point
        self.radius = radius
    
    def update(self, home_pos):
      
        self.home_pos = home_pos
    
    def compute(self, object_position: list[float]) -> list[float]:
        
        d_y = object_position[1] - self.home_pos[1]
        d_x = object_position[0] - self.home_pos[0]
        
        p = util.norm(d_x, d_y)
        
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
        
    def compute(self, d_x: float, d_y: float):
        
        y_l = d_y + self.radius
        y_r = d_y - self.radius

        phi_ccw = self.h_field_cw.compute(d_x, d_y - self.radius)
        phi_cw = self.h_field_ccw.compute(d_x, d_y + self.radius)

        nh_ccw = Nh(phi_ccw)
        nh_cw = Nh(phi_cw)
        # The absolute value of y_l and y_r was not specified in the article, but the obtained results 
        # with this trick are closer to the article images
        spiral_merge = (abs(y_l) * nh_ccw + abs(y_r) * nh_cw) / (2 * self.radius) 

        if -self.radius <= d_y < self.radius:
            phi_tuf = math.atan2(spiral_merge[1], spiral_merge[0])
        elif d_y < -self.radius:
            phi_tuf = self.h_field_ccw.compute(d_x, d_y - self.radius)
        else:
            phi_tuf = self.h_field_cw.compute(d_x, d_y + self.radius)

        return util.wrapToPi(phi_tuf)