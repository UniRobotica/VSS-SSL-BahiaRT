import os
import socket
import struct
import json
import time
import threading

from collections import deque
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
        self._frame_times   =   deque(maxlen=60)
        self._fps           =   0
        
    def printInfo(self):
        
        print(f'FRAME: {self.frame}')
        print('')
        
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
    
    def set_fps(self):
        self._frame_times.append(time.time())
        if len(self._frame_times) <= 3:
            return
        fps_frame_by_frame = [
            (v - i) for i, v in zip(self._frame_times, list(self._frame_times)[1:])
        ]
        self._fps = len(fps_frame_by_frame)/sum(fps_frame_by_frame)

    
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
            self.set_fps()
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))['frame']
            self. frame.__setitem__('fps', self._fps)
    
    def send_data(self, robot_list: list[Robot]):
        """
        Send data for the server.
        
        Args:
            robot_id (int): The robot id
            team_color (bool): The color of the robot team
            wl (float): Left wheel angular velocity
            wr (float): Right wheel angular velocity
        """

        robot_commands = []
        for robot in robot_list :
            robot_commands.append(
                {
                    "robot_id": robot.robot_id,
                    "color": robot.team_color_str(),
                    "wheel_left": robot.wl,
                    "wheel_right": robot.wr,
                }
            )

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