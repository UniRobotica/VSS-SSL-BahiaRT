import socket
import struct
import json
import time
import threading
from collections import deque
from google.protobuf.json_format import MessageToJson
from protocols.ssl_vision import messages_robocup_ssl_wrapper_pb2

class Vision(threading.Thread):
    def __init__(self):
        super(Vision, self).__init__()

        self.frame = {}
        self.last_frame = {}
        
        self.vision_port = 10015
        self.host = '224.5.23.2'

    def run(self):
        print("Starting vision...")
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        print("Vision completed!")

        while True:
            env = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
            data = self.vision_sock.recv(1024)
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))
    
    def _wait_to_connect(self):
        self.vision_sock.recv(1024)
    
    def _create_socket(self):
        print(f"Creating socket with address: {self.host} and port: {self.vision_port}")
        sock = socket.socket(
            socket.AF_INET, 
            socket.SOCK_DGRAM, 
            socket.IPPROTO_UDP
        )

        sock.setsockopt(
            socket.SOL_SOCKET, 
            socket.SO_REUSEADDR, 1
        )

        sock.bind((self.host, self.vision_port))

        mreq = struct.pack(
            "4sl",
            socket.inet_aton(self.host),
            socket.INADDR_ANY
        )

        sock.setsockopt(
            socket.IPPROTO_IP, 
            socket.IP_ADD_MEMBERSHIP, 
            mreq
        )

        return sock

if __name__ == "__main__":
    v = Vision()

    v.start()

    while True:
        time.sleep(1)
        print(' ')
        print(v.frame)