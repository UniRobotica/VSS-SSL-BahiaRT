import time
import math
from utils import util
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

from entities.Robot import Robot
from entities.Ball import Ball

robot = Robot(
    env=args.env,
    robot_id=0,
    team_color=True,   
)
ball = Ball(
    env=args.env  
)

# Criando campo potencial -----------------------------------------------------

from algorithms import univector_field

pot_field = univector_field.MoveToGoalField(
    home_point = [0, 0],
    env = args.env
)

# -----------------------------------------------------------------------------
POWER_MULTIPLY = 1000

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
                pot_field.update_home_point(ball.position)

                angle_to_reach = util.wrap_to_pi(math.atan2(0 - ball.position[1], 0.85 - ball.position[0]))
                print(f'Angle to reach: {math.degrees(angle_to_reach)}')
                
                # Calculando velocidade das rodas
                vector_speed = pot_field.Nh(robot.front_position, angle_to_reach)
                wheels_speed = robot.global_to_ws(vector_speed) * POWER_MULTIPLY
                robot.set_desired(wheels_speed)
                
                robot.print_info()
                
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
                    #robot.set_desired([0,0])
                    vision.send_data([robot])
     
        else:
            print('Waiting for vision data...')
            time.sleep(3)