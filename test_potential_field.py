from entities.Ball import Ball
from entities.Robot import Robot

import time
import argparse

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Para o teste real
if args.env == 'real':
    
    from vision import visionReal
    from comm import serial

    # Criando comunicação
    vision = visionReal.VisionReal()
    serial_comm = serial.SerialComm()
    serial_comm.start()

# Para o teste simulado
else:
    
    from vision import visionSim

    # Criando comunicação
    vision = visionSim.VisionSim()

from algorithms import robot_kinematics

# Criando entidades

robot = Robot(
    env=args.env,
    robot_id=7,
    team_color=False,   
)
ball = Ball(
    env=args.env
)

# Inicializando comunicação
vision.start()

# Criando campo potencial
from algorithms import univector_field

pot_field = univector_field.PointField(
    ball.position,
    lambda x:x,
)

if __name__ == '__main__':
    
    while True:

        time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
        
        if vision.frame:
                
            print('Vision data received')
            
            while True:
                
                time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
                
                # Atualizando informações
                robot.update(vision.frame)
                ball.update(vision.frame)
                pot_field.home_point = ball.position
                
                #Campo potencial
                robot.set_desired(
                    robot_kinematics.global_to_ws(
                        pot_field.getForce(robot.position),
                        robot.orientation
                    ) * 100
                )
                
                print('wl:',robot.wl)
                print('wr:',robot.wr)
                print(' ')
                
                # Enviando informações via Serial
                if args.env == 'real':
                    serial_comm.send(
                        [
                            {
                                'robot_id': robot.robot_id,
                                'wheel_left': robot.wl,
                                'wheel_right': robot.wr,
                                'color': robot.team_color_str()
                            }
                        ]
                    )
                else:
                    vision.send_data(robot)
     
        else:
            print('Waiting for vision data...')
            time.sleep(3)