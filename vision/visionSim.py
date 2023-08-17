import os
import socket
import struct
import json
import time
import threading

from google.protobuf.json_format import MessageToJson
from protocols import packet_pb2, command_pb2

from utils import config
from entities.Robot import Robot

class VisionSim(threading.Thread):
    '''
    A class that contains all communication information from FIRASim. Allows to send and receive data.
    '''

    def __init__(self) -> None:
        super(VisionSim, self).__init__()
        
        sim_config = config.get_config('simulation_config.json')['network']
        
        self.host_address   =   sim_config['host_address']
        self.host_port      =   sim_config['host_port']
        self.listen_address =   sim_config['listen_address']
        self.listen_port    =   sim_config['listen_port']
        self.frame          =   None
        self.last_frame     =   None
        
    def _create_socket(self):
        '''
        Creates a socket.
        
        Args:
            address: the host addrress
            port: the host port
            
        Returns:
            An socket.
        '''
        print(f"Creating socket with address: {self.host_address} and port: {self.host_port}")
        sock = socket.socket(
            socket.AF_INET, 
            socket.SOCK_DGRAM, 
            socket.IPPROTO_UDP 
        )

        sock.setsockopt(
            socket.SOL_SOCKET, 
            socket.SO_REUSEADDR, 1
        )

        sock.bind((self.host_address, self.host_port))

        mreq = struct.pack(
            "4sl",
            socket.inet_aton(self.host_address),
            socket.INADDR_ANY
        )

        sock.setsockopt(
            socket.IPPROTO_IP, 
            socket.IP_ADD_MEMBERSHIP, 
            mreq
        )

        return sock
    
    def _wait_to_connect(self):
        self.vision_sock.recv(1024)
    
    def run(self):
        print("Starting simulated vision...")
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        print("Simulated vision completed!")
        
        while True:
            env = packet_pb2.Environment()
            data = self.vision_sock.recv(1024)
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))['frame']
    
    def send_data(self, robot: Robot):
        """
        Send data for the server.
        
        Args:
            robot_id (int): The robot id
            team_color (bool): The color of the robot team
            wl (float): Left wheel angular velocity
            wr (float): Right wheel angular velocity
        """


        robot_commands = [
        {
            "robot_id": robot.robot_id,
            "color": robot.team_color_str(),
            "wheel_left": robot.wl,
            "wheel_right": robot.wr,
        }]

        commands = command_pb2.Commands()

        for robot in robot_commands:
            command = commands.robot_commands.add()
            command.yellowteam = True if robot['color'] == 'yellow' else False
            command.wheel_right = robot['wheel_right']
            command.wheel_left = robot['wheel_left']
            command.id = robot['robot_id']

        packet = packet_pb2.Packet()
        packet.cmd.CopyFrom(commands)

        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(
            packet.SerializeToString(), 
            (self.listen_address, self.listen_port)
        )


if __name__ == "__main__":
    
    vision = VisionSim()
    vision.setup()

    while True:
        vision.update()
        print(vision.frame)
        time.sleep(0.100)
        os.system("clear")