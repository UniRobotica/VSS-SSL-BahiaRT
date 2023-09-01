from abc import ABC, abstractmethod
import numpy as np
import math

from utils import util

CONSTANTS = {
    'simulation': {
        'D_E': 0.1,
        'K_P': 0,
        'K_O': 0,
        'D_MIN': 0,
        'DELTA' : 0
    },
    'real': {
        'D_E': 0,
        'K_P': 0,
        'K_O': 0,
        'D_MIN': 0,
        'DELTA' : 0
    }
}

def Nh(phi: float) -> list[float]:
    
    return np.array([math.cos(phi), math.sin(phi)])


class BaseField(ABC):
    
    def __init__(
        self, 
        home_point: list[float]
    ) -> None:
        
        self.home_point = home_point
    
    def update_home_point(self, home_point: list[float]):
        
        self.home_point = home_point
    
    @abstractmethod
    def compute(self, object_position: list[float]) -> float:
        pass
    
    def Nh(self, object_position) -> list[float]:
        
        return Nh(self.compute(object_position))


class HyperbolicField(BaseField):
    '''
    
    Um campo potencial hiperbólico que pode ter uma força de sentido horário ou anti-horário.
    '''    
    
    def __init__(
        self, 
        home_point: list[float],
        cw: bool,
        env: str
    ) -> None:
        
        super().__init__(home_point)
        
        self.env = env
        self.cw = not cw
        self.radius = CONSTANTS[self.env]['D_E']
        self.kp = CONSTANTS[self.env]['K_P']
    
    def compute(self, object_position: list[float]) -> float:
        
        dx, dy = util.delta(self.home_point, object_position)
        
        theta = math.atan2(dy, dx)
        rho = util.norm(dx, dy)
        
        if rho > self.radius:
            angle = (math.pi / 2) * (2 - ((self.radius + self.kp) / (rho + self.kp)))
        elif 0 <= rho <= self.radius:
            angle = (math.pi / 2) * math.sqrt(rho / self.radius)

        if self.cw:
            return util.wrap_to_pi(theta + angle)
        else:
            return util.wrap_to_pi(theta - angle)
    
    def compute_for_tuf(self, dx: float, dy: float) -> float:
        
        theta = math.atan2(dy, dx)
        rho = util.norm(dx, dy)
        
        if rho > self.radius:
            angle = (math.pi / 2) * (2 - ((self.radius + self.kp) / (rho + self.kp)))
        elif 0 <= rho <= self.radius:
            angle = (math.pi / 2) * math.sqrt(rho / self.radius)

        if self.cw:
            return util.wrap_to_pi(theta + angle)
        else:
            return util.wrap_to_pi(theta - angle)
        

class RepulsivePointField(BaseField):
    '''
    Um campo potencial de atração originário de um único ponto.
    '''
    
    def __init__(
        self, 
        home_point: list[float],
        env: str
    ) -> None:
        
        super().__init__(home_point)
        
        self.env = env
    
    def compute(self, object_position: list[float]) -> list[float]:
        
        dx, dy = util.delta(self.home_point, object_position)
        
        return math.atan2(dy, dx)
        

class MoveToGoalField(BaseField):
    
    def __init__(
        self,
        home_point: list[float],
        env
    ) -> None:
    
        super().__init__(home_point)
        
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
        
    def compute(self, object_position: list[float]) -> float:
        
        dx, dy = util.delta(self.home_point, object_position)
        
        yl = dy + self.radius
        yr = dy - self.radius

        phi_cw = self.h_field_cw.compute_for_tuf(dx, dy - self.radius)
        phi_ccw = self.h_field_ccw.compute_for_tuf(dx, dy + self.radius)

        nh_ccw = Nh(phi_ccw)
        nh_cw = Nh(phi_cw)

        spiral_merge = (abs(yl) * nh_ccw + abs(yr) * nh_cw) / (2 * self.radius) 

        if dy > self.radius:
            phi_tuf = phi_cw
        elif dy < -self.radius:
            phi_tuf = phi_ccw
        else:
            phi_tuf = math.atan2(spiral_merge[1], spiral_merge[0])
            
        return util.wrap_to_pi(phi_tuf)
    

class AvoidObstacleField(BaseField):
    
    def __init__(
        self,
        home_point: list[float]
    ) -> None:
        
        super().__init__(home_point)
        