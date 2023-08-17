'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import numpy as np
from types import FunctionType

from utils import util
from entities.Robot import Robot

class PointField():
    """
    Represents a potential field that has a point as the origin
    """
    
    def __init__(
        self,
        home_point: list[float], 
        decay: FunctionType = lambda x:1, 
        max_radius: float = 1000.0,
        type: bool = True, # True: attractive | False: repulsive
    ) -> None:
        
        self.home_point = home_point
        self.decay = decay
        self.max_radius = max_radius
        self.type = type
        
    def getForce(self, target_position, speed=100) -> list[float]:
        """
        Gets the force in a point based on a simulated potential field that will constantly reduce the speed of the robot as the robot approaches the center of the force. 
        The potential field is limited by the maximum radius and the speed decay is based on a decay function that is given as a parameter.

        Args:
            target_position (list[float]): [x,y].
            speed (float, optional): The vector speed. Defaults to 100.

        Returns:
            list[float]: an array [dx, dy], representing the potential field force at the point where the robot is
        """
        
        if self.type:

            to_target = [
                - self.home_point[0] + target_position[0], - self.home_point[1] + target_position[1]
            ]

            to_target_scalar = np.linalg.norm(to_target)
            to_target_scalar_norm = max(0, min(1, to_target_scalar/self.max_radius))
            
            force = self.decay(to_target_scalar_norm)
            to_target_norm = util.unit_vector(to_target)
            
            return [
                    to_target_norm[0] * force * speed,
                    to_target_norm[1] * force * speed
            ]
        
        else:
            # Not implemented
            pass
