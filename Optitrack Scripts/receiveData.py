import logging
import math
from threading import Thread
import time
import json
import numpy as np
import socket
import struct


OPTI_PORT = '1511'
OPTI_IP = '192.168.0.99'
CLIENT_PORT = '3500'

def receive_data():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0', int(CLIENT_PORT)))
        while True:
            data = s.recvfrom(1024)[0]
            if not data:
                print('No data received')
                break
            else:
                print(data)
                [x,y,z, opti_x, opti_y, opti_z,  opti_w,roll, yaw, pitch, bodyID, framecount] = struct.unpack('f\ffffffffffff', data)
                print(x, y, z)


if __name__ == '__main__':
    receive_data()