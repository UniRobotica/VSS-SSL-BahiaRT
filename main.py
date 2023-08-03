from entities.Ball import Ball
from entities.Robot import Robot

from vision import vision
from comm import serial

import time

from strategy import potential_field

vision_rl = vision.Vision()
serial_comm = serial.SerialComm()

vision_rl.start()
serial_comm.start()

robot_blue = Robot(team_color=False)
ball = Ball()

pot_field = potential_field.PointField(
    ball.position,
    lambda x:x**2,
    max_radius=100
)

if __name__ == '__main__':
    
    while True:

        time.sleep(0.003)
        
        if vision_rl.frame.get('detection', None) != None:

            robot_blue.update(vision_rl.frame)
            ball.update(vision_rl.frame)
            
            pot_field.home_point = ball.position
            
            wl, wr = robot_blue.clever_trick(
                pot_field.getForce(robot_blue),
                consider_back=True
            )
            
            serial_comm.send(
                [
                    {
                        'robot_id': robot_blue.robot_id,
                        'wheel_left': wl,
                        'wheel_right': wr,
                        'color': 'blue' if robot_blue.team_color else 'yellow'
                    }
                ]
            )
        
    
    