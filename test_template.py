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

from strategy import clever_trick

# Criando entidades

robot = Robot(
    clever_trick.CleverTrick(),
    env=args.env,
    team_color=True,   
)
ball = Ball(
    env=args.env
)

# Inicializando comunicação
vision.start()

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
                
                #Exemplo de comando
                desired_speed = [100, 100]
                robot.set_desired()
                
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