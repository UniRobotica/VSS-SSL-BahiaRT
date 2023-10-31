import math
import numpy as np

from entities.Robot import Robot
from utils import util


def speed_to_power(v: float, w: float):
    """
    Returns the wheels angular speed

    Args:
        v (float): Linear speed
        w (float): Angular speed

    Returns:
        [wl, wr](float): wl - Left wheel angular speed, wr - Right wheel angular speed
    """
    wl = (2 * v - w * Robot.L)/2 * Robot.R
    wr = (2 * v + w * Robot.L)/2 * Robot.R
    
    return wl, wr
       
        
def calculate_local_speeds(vg: list[float], orientation: float):
    
    #proporcional angular
    n = (1/0.1)
    
    theta = util.apply_angular_decay(orientation, 1)
    
    v = vg[0] * math.cos(-theta) - vg[1] * math.sin(-theta)
    w = n * (vg[0] * math.sin(-theta) + vg[1] * math.cos(-theta))
    
    #inversed front side
    # desired_angle_rad = math.atan2(vg[1], vg[0])
    # desired_angle_deg = util.convert_to_deg(desired_angle_rad)

    # robot_angle_deg = util.convert_to_deg(orientation)
    # robot_angle_deg_inverted = util.add_deg(robot_angle_deg, 180)
    
    # angle_error_1 = desired_angle_deg - robot_angle_deg
    # angle_error_2 = desired_angle_deg - robot_angle_deg_inverted
    
    # if abs(angle_error_2) < abs(angle_error_1):
    #    w *= -1
    
    return v, -w


def global_to_ws(vg: list[float], orientation: float):
    """
    Gives the wheels power to reach the configuration [vx, vy, omega].

    Args:
        vg (list[float]): Global speed [vx, vy]
        orientation (float): Global orientation [omega]

    Returns:
        list(float): Wheels power [wl, wr]
    """
    
    v, w = calculate_local_speeds(vg, orientation)
    return np.array(speed_to_power(v, w))