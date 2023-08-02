from entities.Ball import Ball
from entities.Robot import Robot

from vision import vision
from comm import serial

from strategy import potential_field

vision_rl = vision.Vision()
serial_comm = serial.SerialComm()

if __name__ == '__main__':
    
    robot_blue = Robot()
    ball = Ball()
    
    vision_rl.start()
    serial_comm.start()
    
    while True:

        robot_blue.update(vision_rl.frame)
        ball.update(vision_rl.frame)
        
        wl, wr = robot_blue.clever_trick(
            potential_field.PointField(
                ball.position,
                lambda x:x**2,
                max_radius=100
            )
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
    
    