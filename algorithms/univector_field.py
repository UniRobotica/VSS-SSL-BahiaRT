from abc import ABC, abstractmethod
import numpy as np
import math

from utils import util

CONSTANTS = {
    'simulation': {
        'D_E': 0.1,
        'K_R': 0,
        'K_O': 1,
        'D_MIN': 0,
        'DELTA' : 0
    },
    'real': {
        'D_E': 0,
        'K_R': 0,
        'K_O': 1,
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


class AttractionField(BaseField):
    """
    Represents a potential field that has a point as the origin
    """
    
    def __init__(
        self,
        home_point: list[float],
    ) -> None:
        super().__init__(home_point)
        
    def compute(self, object_position: list[float]) -> list[float]:

        delta = util.delta(object_position, self.home_point)
        to_target_norm = util.unit_vector(delta)
        
        return math.atan2(to_target_norm[1], to_target_norm[0])
    
    def Nh(self, object_position: list[float]):
        
        delta = util.delta(self.home_point, object_position)
        return util.unit_vector(delta)
        
        
class EdgeField():
    
    HOME_POSITION = [0,0]
    
    def __init__(
        self, 
        field_size: list[float],
        range: float,
        env: str='simulation'
    ) -> None:
        
        self.x = [-field_size[0] / 2, field_size[0] / 2]
        self.y = [-field_size[1] / 2, field_size[1] / 2]
        
        self.range = range
        self.env = env
        
        self.a_field = AttractionField(
            self.HOME_POSITION
        )
        
    def compute(self, object_position):
        
        if (self.x[0] + self.range) < object_position[0] < (self.x[1] - self.range) and (self.y[0] + self.range) < object_position[1] < (self.y[1] - self.range):
            
            return None
        
        else:
            
            return self.a_field.compute(object_position)
        
    def Nh(self, object_position) -> list[float]:
        
        phi = self.compute(object_position)
        
        if phi == None:
            return [0,0]
        else:
            return Nh(phi)


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
    
    @staticmethod
    def compute_for_auf(dx: list[float], dy: list[float]):
        
        return math.atan2(dy, dx)
        

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
        self.kp = CONSTANTS[self.env]['K_R']
    
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

        spiral_merge = (abs(yl) * nh_cw + abs(yr) * nh_ccw) / (2 * self.radius) 

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
        home_point: list[float],
        env = 'simulation'
    ) -> None:
        
        super().__init__(home_point)
        
        self.home_speed: list[float] = 0
        self.ko = CONSTANTS[env]['K_O']
        
    def update_home_point(self, home_point: list[float], home_speed):
        
        super().update_home_point(home_point)
        self.home_speed = np.array(self.home_speed)
        self.rep_field.update_home_point(self.home_speed)
        
    def compute(self, object_position: list[float], object_speed: list[float]):
        
        obstacle_position = np.array(obstacle_position)
        object_speed = np.array(object_speed)

        s_vec = self.ko * (self.home_speed - object_speed)
        s_norm = util.norm(s_vec[0], s_vec[1])
        
        dx, dy = util.delta(object_position, self.home_point)
        obs_robot_dist = util.norm(dx, dy)

        if obs_robot_dist >= s_norm:
            p_line_obs = obstacle_position + s_vec
        else:
            p_line_obs = obstacle_position + obs_robot_dist * s_vec / s_norm

        dx2, dy2 = util.delta(p_line_obs, object_position)
        
        phi = RepulsivePointField.compute_for_auf(dx2, dy2)
        
        return util.wrap_to_pi(phi)
        