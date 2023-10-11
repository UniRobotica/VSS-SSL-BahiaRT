import argparse
from utils import config

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Criando comunicação ----------------------------------------------------------

# Para o teste real
    
from comm import serial

# Criando comunicação
serial_comm = serial.SerialComm()
serial_comm.start()

# Criando entidades -----------------------------------------------------

from entities.Robot import Robot

# Definindo informação dos robôs ----------------------------------------

robot_ids = [0, 1, 2]
team_side = True

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

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    
    wl = -50
    wr = 50
    
    while True:

        serial_comm.send([
            {
                'robot_id': list_robots[0].robot_id,
                'wheel_left': wl,
                'wheel_right': wr,
                'color': list_robots[0].team_color_str()
            },
            {
                'robot_id': list_robots[1].robot_id,
                'wheel_left': wl,
                'wheel_right': wr,
                'color': list_robots[1].team_color_str()
            },
            {
                'robot_id': list_robots[2].robot_id,
                'wheel_left': wl,
                'wheel_right': wr,
                'color': list_robots[2].team_color_str()
            }
        ])