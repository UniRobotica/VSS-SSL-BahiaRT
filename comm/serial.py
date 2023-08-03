import os
from serial import Serial
import utils.util as util

class SerialComm(object):
    def __init__(self):
        super(SerialComm, self).__init__()
        self.config = util.get_config('real_life_config.json')
        self.commands = []
        self.command_port = os.environ.get('COMM_PORT', self.config['comm']['comm_port'])
        self.baud_rate = int(os.environ.get('BAUD_RATE', self.config['comm']['baud_rate']))

    def start(self):
        print("Starting communication...")
        self.comm = Serial(self.command_port, self.baud_rate)
        print(f"Communication port created on {self.command_port}!")
    
    def send(self, robot_commands = []):
        '''
        Send commands to ESP-32

        robot_commands follows:
        [
            {
                robot_id: int,
                color: 'yellow|blue',
                wheel_left: float,
                wheel_right: float,
            }
        ]
        '''
        message = "<"
        robot_commands = sorted(robot_commands, key = lambda i: i['robot_id'])
        for rb in robot_commands:
            message += f"{rb['robot_id']},{round(rb['wheel_left'], 2)},{round(rb['wheel_right'], 2)},"

        message = message[:-1] + '>'
        print(message)
        self.comm.write(message.encode())

    def _get_robot_color(self, robot):
        return True if robot['color'] == 'yellow' else False

if __name__ == "__main__":
    import time
    c = SerialComm()

    c.start()

    while True:
        time.sleep(1)
        c.send(
            [
                {
                    'robot_id': 0,
                    'wheel_left': 20,
                    'wheel_right': -20,
                    'color': 'blue'
                }
            ]
        )