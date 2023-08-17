'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import numpy as np

def unit_vector(vector: list[float]) -> list[float]:
    """
    Calculates the vector normalization and, from that, calculates the unit vector.

    Returns:
        list[float]: An unit vector [x, y]
    """
    
    if np.linalg.norm(vector) == 0:
        return np.array([0, 0])
    return vector / np.linalg.norm(vector)

def apply_angular_decay(angular_velocity: float, decay_rate: float) -> float:
    return angular_velocity * decay_rate

def detect_ball_proximity(ball_position: list[float], robot_position: list[float]) -> float:
    distance = np.linalg.norm(np.array(ball_position) - np.array(robot_position))
    return distance

def convertTodeg(rad: float) -> float:
    return np.rad2deg(rad) + 180
    
def add_deg(deg1:float, deg2: float) -> float:
    result = deg1 + deg2
    
    if result >= 360:
        return result - 360
    
    return result