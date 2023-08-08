from strategy import potential_field
from entities.Ball import Ball
from entities.Robot import Robot

from vision import vision
from comm import serial

import time

# Criando comunicação
vision_rl = vision.Vision()
serial_comm = serial.SerialComm()

# Criando entidades
robot_blue = Robot(team_color=False)
ball = Ball()

# Criando campo potencial
pot_field = potential_field.PointField(
    ball.position,
    lambda x:x**2,
    max_radius=100
)

# Inicializando comunicação
vision_rl.start()
serial_comm.start()

if __name__ == '__main__':
    
    while True:

        time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
        
        if vision_rl.frame.get('detection', None) != None:
            
            # Atualizando informações recebidas via Visão
            robot_blue.update(vision_rl.frame)
            ball.update(vision_rl.frame)
            
            pot_field.home_point = ball.position
            
            # Calculando velocidades angulares das rodas do robô
            wl, wr = robot_blue.clever_trick(
                pot_field.getForce(robot_blue, 10000),
                consider_back=True
            )
            
            # Enviando informações via Serial
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
        
    
    