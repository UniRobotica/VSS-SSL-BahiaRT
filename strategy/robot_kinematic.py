import math

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
        
def calculate_local_speeds(self, v_g: list[float], orientation: float):
    
    #proporcional angular
    n = (1/0.185)
    
    theta = util.apply_angular_decay(orientation, 1)
    
    v = v_g[0] * math.cos(-theta) - v_g[1] * math.sin(-theta)
    w = n * (v_g[0] * math.sin(-theta) + v_g[1] * math.cos(-theta))
    
    #inversed front side
    #desired_angle_rad = math.atan2(vector_speed[1], vector_speed[0])
    #desired_angle_deg = util.convert_to_deg(desired_angle_rad)

    #robot_angle_deg = util.convert_to_deg(orientation)
    #robot_angle_deg_inverted = util.add_deg(robot_angle_deg, 180)
    
    #angle_error_1 = desired_angle_deg - robot_angle_deg
    #angle_error_2 = desired_angle_deg - robot_angle_deg_inverted
    
    #if abs(angle_error_2) < abs(angle_error_1):
    #    w *= -1
    
    return v, w

def ddr_ik(vx, omega, L=0.5, r=0.1):
    """DDR inverse kinematics: calculate wheels speeds from desired velocity."""
    return (vx - (L/2)*omega)/r, (vx + (L/2)*omega)/r

def ddr_fk(phidot_L, phidot_R, L=0.5, r=0.1):
    """DDR inverse kinematics: calculate wheels speeds from desired velocity."""
    return (phidot_R+phidot_L)*r/2, (phidot_R-phidot_L)*r/L