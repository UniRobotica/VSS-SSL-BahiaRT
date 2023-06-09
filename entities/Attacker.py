from geometry.Pose import Pose
from geometry.Point import Point
from geometry.Vector import Vector
from .Player import Player
import math

class Attacker(Player):
    
    def __init__(
        self, 
        position: Pose = Pose(),
        velocity: Vector = Vector(),
        robot_id: int = 0, 
        team_color: bool = True
    ) -> None:
        super().__init__(position, velocity, robot_id, team_color)
        
    def shoot(self, ball_pos: Point, target_pos: Point, robot_speed: float = 50) -> list[float]:
        """
        This function of the angular speeds of the wheels to ensure that the robot traces a favorable route for the kick.

        Args:
            ball_pos (Point): Ball position values [x,y]
            target_pos (Point): Target position values [x,y]
            robot_speed (float, optional): Desired robot velocity. Defaults to 50.

        Returns:
            list[float]: The angular speeds of the wheels [wl, wr]
        """
        
        dx1 = target_pos.x - ball_pos.x
        dy1 = target_pos.y - ball_pos.y
        
        if dx1 == 0 and dy1 == 0:
            angle1 = 90
        else:
            angle1 = int(180 / math.pi * math.atan2(dy1, dx1))
        
        robot_pos = self.position
        dx2 = ball_pos.x - robot_pos.x
        dy2 = ball_pos.y - robot_pos.y
        
        if dx2 == 0 and dy2 == 0:
            angle2 = 90
        else:
            angle2 = int(180 / math.pi * math.atan2(dy2, dx2))
        
        theta_d = 2 * angle2 - angle1
        theta_e = theta_d - robot_pos.theta
        
        while theta_e > 180:
            theta_e -= 360
        while theta_e <= -180:
            theta_e += 360
            
        if theta_e < -90:
            theta_e += 180
            robot_speed *= -1
        elif theta_e > 90:
            theta_e -= 10
            robot_speed *= -1
        
        if abs(theta_e) > 50:
            kA = 0.16
        elif abs(theta_e) > 40:
            kA = 0.18
        elif abs(theta_e) > 30:
            kA = 0.2
        elif abs(theta_e) > 20:
            kA = 0.22
        else:
            kA = 0.24
        
        wl = (robot_speed - kA * theta_e) / self.R
        wr = robot_speed + kA * theta_e / self.R
        
        return [wl, wr]
        