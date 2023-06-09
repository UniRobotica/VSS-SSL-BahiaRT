'''
Based on NeonFC algorithm.

Modified by:    - Luís Henrique
                - Lucas

Date: 15/06/2023
'''

import os
import socket
import struct
import json
import time
import utils.util as util

from collections import deque
from google.protobuf.json_format import MessageToJson
from protocols.ssl_vision import messages_robocup_ssl_wrapper_pb2


class Vision():
    def __init__(
        self,
        host_address: str = '224.5.23.2',
        host_port: int = 10015,
        config: dict = util.get_config('config.json'),
    ):
        self.config       =   config
        self.frame        =   {}
        self.last_frame   =   {}
        self.host_port    =   host_port
        self.host_address =   host_address
        self._fps         =   0
        self._frame_times =   deque(maxlen=60)

    def _create_socket(self):
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

    def setup(self):
        print("Starting vision...")
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        print("Vision completed!")
    
    def update(self):
        self._wait_to_connect()
    
        env = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        data = self.vision_sock.recv(1024)
        self.set_fps()
        env.ParseFromString(data)
        self.frame = json.loads(MessageToJson(env))


if __name__ == "__main__":
    import time
    
    vision = Vision()
    vision.setup()

    while True:
        vision.update()
        print(vision.frame)
        time.sleep(0.100)
        print(" ")
        #os.system("clear")