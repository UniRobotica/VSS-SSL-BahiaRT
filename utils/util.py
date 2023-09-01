'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import math
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

def norm(d_x: int, d_y: int) -> float:
    
    return math.sqrt(d_x ** 2 + d_y ** 2)

def wrap_to_pi(angle: float) -> float:
    
    if angle > math.pi:
        return angle - 2 * math.pi
    if angle < -math.pi:
        return 2 * math.pi + angle
    else:
        return angle

def apply_angular_decay(angular_velocity: float, decay_rate: float) -> float:
    
    return angular_velocity * decay_rate

def detect_ball_proximity(ball_position: list[float], robot_position: list[float]) -> float:
    
    distance = np.linalg.norm(np.array(ball_position) - np.array(robot_position))
    
    return distance

def convert_to_deg(rad: float) -> float:
    
    return np.rad2deg(rad) + 180
    
def add_deg(deg1:float, deg2: float) -> float:
    result = deg1 + deg2
    
    if result >= 360:
        return result - 360
    
    return result

def delta(p1: list[float], p2: list[float]) -> list[float]:
    
    return [
        p2[0] - p1[0],
        p2[1] - p1[1]
    ]

def angleBetweenTwoPoints(p1: list[float], p2: list[float]) -> float:
    
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def createRotationalMatrix(angleInRadians: float) -> list[float]:
    
    c, s = np.cos(angleInRadians), np.sin(angleInRadians)
    return np.array(((c, -s), (s, c)))

def rotateAngle(angleInRadians: list[float]) -> list[float]:
    
    return np.dot(
        createRotationalMatrix(np.deg2rad(180)),
        [math.cos(angleInRadians), math.sin(angleInRadians)]
    )