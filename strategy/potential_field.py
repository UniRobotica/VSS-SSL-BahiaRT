'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import numpy as np
from geometry.Vector import Vector
from geometry.Point import Point
from types import FunctionType
from geometry.Vector import Vector
from entities.Player import Player

class PointField():
    """
    Represents a potential field that has a point as the origin
    """
    
    def __init__(
        self,
        home_point: Point, 
        decay: FunctionType = lambda x:1, 
        max_radius: float = 1000.0,
        type: bool = True, #True: attractive | False: repulsive
    ) -> None:
        
        self.home_point = home_point
        self.decay = decay
        self.max_radius = max_radius
        self.type = True
        
    def getForce(self, robot: Player, speed=100) -> Vector:
        """
        Gets the force in a point based on a simulated potential field that will constantly reduce the speed of the robot as the robot approaches the center of the force. 
        The potential field is limited by the maximum radius and the speed decay is based on a decay function that is given as a parameter.

        Args:
            robot (PrimitivePlayer): A robot player class.
            speed (float, optional): The vector speed. Defaults to 100.

        Returns:
            Vector: an array [dx, dy], representing the potential field force at the point where the robot is
        """

        to_target = Vector()
        to_target.setValues(
            np.subtract(self.home_point.values(), robot.position.getPointValues())
        )

        to_target_scalar = np.linalg.norm(to_target.values())
        to_target_scalar_norm = max(0, min(1, to_target_scalar/self.max_radius))
        
        force = self.decay(to_target_scalar_norm)
        to_target_norm = Vector.unit_vector(to_target)
        return Vector(
                to_target_norm[0] * force * speed,
                to_target_norm[1] * force * speed
        )


def attraction_potential(
    target_pos: Point,
    robot_pos: Point, 
    scaling_param: float = 1.0, 
    target_zone_radius: float = 1.0
) -> Vector:
    
    square_distance = (robot_pos.x - target_pos.x)**2 + (robot_pos.y - target_pos.y)**2
    distance = square_distance**0.5
    
    #if square_distance > target_zone_radius:
    #    attraction_force = (target_zone_radius * scaling_param * square_distance) - (0.5 * scaling_param * (target_zone_radius**2))
    #else:
    #    attraction_force = 0.5 * scaling_param * distance
    
    vector = np.subtract(robot_pos.values(), target_pos.values())
    
    if square_distance > target_zone_radius:
        vector = np.multiply( vector, (scaling_param / distance * -1) )
    else:
        vector = np.multiply( vector, scaling_param * -1 )
    
    return Vector(vector[0], vector[1])
    
def repulsive_potential(
    home_pos: Point, 
    robot_pos: Point, 
    scaling_param: float = 1.0, 
    distance_threshold: float = 1.0
) -> Vector:
    
    square_distance = (robot_pos.x - home_pos.x)**2 + (robot_pos.x - home_pos.x)**2
    distance = square_distance**0.5
    
    #if distance <= distance_threshhold:
    #    repulsive_force = (0.5 * scaling_param) * (((distance**-1) - (distance_threshhold**-1))**2)
    #else: 
    #    repulsive_force = 0

    vector = np.subtract(robot_pos.values(), home_pos.values())
    
    if distance <= distance_threshold:
        vector = np.multiply(
            vector,
            (scaling_param * ((distance**-1) - (distance_threshold**-1))**2 * (square_distance**-1))
        )
    else: 
        vector = [0, 0]
    
    return Vector(vector[0], vector[1])