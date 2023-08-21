from entities.Ball import Ball
from entities.Robot import Robot

import time
import argparse

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Criando comunicação ----------------------------------------------------------

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

# Inicializando comunicação
vision.start()

# Criando entidades -----------------------------------------------------

from strategy import clever_trick

robot = Robot(
    clever_trick.CleverTrick(consider_back=False),
    env=args.env,
    team_color=True,   
)
ball = Ball(
    env=args.env
)

# Criando campo potencial -----------------------------------------------------
from strategy import potential_field

pot_field = potential_field.HyperbolicField(
    [0, 0],
    0.025,
    direction=True
)

# -----------------------------------------------------------------------------
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
                
                #Campo potencial
                robot.set_desired(
                    pot_field.getForce(robot.position, 3000)
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