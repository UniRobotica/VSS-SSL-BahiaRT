import time
import argparse
from utils import config
from pyVSSSReferee.RefereeComm import RefereeComm

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
    referee = RefereeComm('real_life_config.json')
    serial_comm = serial.SerialComm()
    serial_comm.start()

# Para o teste simulado
else:
    
    from vision import visionSim
    referee = RefereeComm('config.json')
    # Criando comunicação
    vision = visionSim.VisionSim()

# Inicializando comunicação
vision.start()

# Criando entidades -----------------------------------------------------

from entities.Robot import Robot
from entities.Ball import Ball
from algorithms import robot_kinematics

# Definindo informação dos robôs ----------------------------------------

robot_ids = [0, 1, 2]
team_side = True

if args.env == 'real':
    
    real_config = config.get_config('real_life_config.json')
    robot_ids = real_config['match']['robot_ids']
    # True for blue team and False for yellow team

robot0 = Robot(
    env=args.env,
    robot_id=robot_ids[0],
    team_color=team_side,
)
robot1 = Robot(
    env=args.env,
    robot_id=robot_ids[1],
    team_color=team_side,   
)
robot2 = Robot(
    env=args.env,
    robot_id=robot_ids[2],
    team_color=team_side,   
)
list_robots = [robot0, robot1, robot2]

ball = Ball(
    env=args.env  
)

# Criando campo potencial -----------------------------------------------------

from algorithms import univector_field

field_config = config.get_config('field.json')
if team_side:
    gk_home_pos = field_config['blue_goal'] 
else:
    gk_home_pos = field_config['yellow_goal']

gk_field = univector_field.HyperbolicField(
    home_point = [0, 0],
    cw=True,
    env=args.env
)
def_field = univector_field.HyperbolicField(
    home_point = [0, 0],
    cw=True,
    env=args.env
)
attacker_field = univector_field.HyperbolicField(
    home_point = [0, 0],
    cw=True,
    env=args.env
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
                for robot in list_robots:
                    robot.update(vision.frame)
                ball.update(vision.frame)
                
                attacker_field.update_home_point(ball.position)
                def_field.update_home_point(ball.position)
                gk_field.update_home_point(ball.position)
                
                POWER_MULTIPLY = 500 if args.env == 'simulation' else 2000
                # GoalKeeper
                robot_vector = gk_field.Nh(list_robots[0].position)
                list_robots[0].set_desired(
                    robot_kinematics.global_to_ws(
                        robot_vector, 
                        robot.orientation
                    ) * POWER_MULTIPLY
                )
                # Defense
                robot_vector = def_field.Nh(list_robots[1].position)
                list_robots[1].set_desired(
                    robot_kinematics.global_to_ws(
                        robot_vector, 
                        robot.orientation
                    ) * POWER_MULTIPLY
                )
                # Attacker
                robot_vector = attacker_field.Nh(list_robots[2].position)
                list_robots[2].set_desired(
                    robot_kinematics.global_to_ws(
                        robot_vector, 
                        robot.orientation
                    ) * POWER_MULTIPLY
                )
                
                for robot in list_robots:
                    print('ID: ', robot.robot_id)
                    print('wl:',robot.wl)
                    print('wr:',robot.wr)
                    print(' ')
                
                # Enviando informações via Serial
                if args.env == 'real':
                    if referee.can_play:
                        serial_comm.send([
                            {
                                'robot_id': robot0.robot_id,
                                'wheel_left': robot0.wr,
                                'wheel_right': robot0.wl,
                                'color': robot0.team_color_str()
                            },
                            {
                                'robot_id': robot1.robot_id,
                                'wheel_left': robot1.wr,
                                'wheel_right': robot1.wl,
                                'color': robot1.team_color_str()
                            },
                            {
                                'robot_id': robot2.robot_id,
                                'wheel_left': robot2.wr,
                                'wheel_right': robot2.wl,
                                'color': robot2.team_color_str()
                            }
                        ])
                    else:
                        serial_comm.send([
                            {
                                'robot_id': list_robots[0].robot_id,
                                'wheel_left': 0,
                                'wheel_right': 0,
                                'color': list_robots[0].team_color_str()
                            },
                            {
                                'robot_id': list_robots[1].robot_id,
                                'wheel_left': 0,
                                'wheel_right': 0,
                                'color': list_robots[1].team_color_str()
                            },
                            {
                                'robot_id': list_robots[2].robot_id,
                                'wheel_left': 0,
                                'wheel_right': 0,
                                'color': list_robots[2].team_color_str()
                            }
                        ])
                    
                else:
                    if referee.can_play:
                        vision.send_data(list_robots)
                    else:
                        for robot in list_robots:
                            robot.wl = 0
                            robot.wr = 0
                        vision.send_data(list_robots)
     
        else:
            print('Waiting for vision data...')
            time.sleep(3)