'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import numpy as np
from types import FunctionType

def unit_vector(vector: list[float]) -> list[float]:
    """
    Calculates the vector normalization and, from that, calculates the unit vector.

    Returns:
        list[float]: An unit vector [x, y]
    """
    
    if np.linalg.norm(vector) == 0:
        return np.array([0, 0])
    return vector / np.linalg.norm(vector)

class PointField():
    """
    Represents a potential field that has a point as the origin
    """
    
    def __init__(
        self,
        home_point: list[float], 
        decay: FunctionType = lambda x:1, 
        max_radius: float = 1000.0,
        type: bool = True, #True: attractive | False: repulsive
    ) -> None:
        
        self.home_point = home_point
        self.decay = decay
        self.max_radius = max_radius
        self.type = True
        
    def getForce(self, robot, speed=100) -> list[float]:
        """
        Gets the force in a point based on a simulated potential field that will constantly reduce the speed of the robot as the robot approaches the center of the force. 
        The potential field is limited by the maximum radius and the speed decay is based on a decay function that is given as a parameter.

        Args:
            robot (): A robot player class.
            speed (float, optional): The vector speed. Defaults to 100.

        Returns:
            list[float]: an array [dx, dy], representing the potential field force at the point where the robot is
        """

        to_target = [
            np.subtract(self.home_point, robot.position)
        ]

        to_target_scalar = np.linalg.norm(to_target)
        to_target_scalar_norm = max(0, min(1, to_target_scalar/self.max_radius))
        
        force = self.decay(to_target_scalar_norm)
        to_target_norm = unit_vector(to_target)
        
        return [
                to_target_norm[0] * force * speed,
                to_target_norm[1] * force * speed
        ]