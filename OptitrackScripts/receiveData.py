
import numpy as np
import socket
import struct
import time

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
                # print(data)
                [x1,z1,y1, x2, z2, y2, x3, z3, y3] = struct.unpack('fffffffff', data)
                # print(x1, y1, z1, x2, y2, z2)

                # Point 1: reference point
                # Point 2: Pivot
                # Point 3: Tip

                angle = -np.degrees(np.arctan((z3-z2)/np.sqrt(((x3-x2)**2) + ((y3-y2)**2))))
                prev_angle = angle

                u1 = np.array([x1, z1, y1])
                u2 = np.array([x2, z2, y2])
                u3 = np.array([x3, z3, y3])


                v1 = u2 - u3 #Pivot to Tip
                v2 = u1 - u3 #Pivot to Reference

                comp = np.dot(v2, v1)/np.linalg.norm(v2)

                # print(angle, comp)
                # print(u1, u2, u3)
                # print(v1, v2)
                # print(x2 - x3, x1 - x3)
                # print(comp)


                if (comp < 0):
                    angle = -angle


                #angle = 90 + angle
                print(angle)


                
            


if __name__ == '__main__':
    receive_data()

