
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

                # Point 1: reference
                # Point 2: Tip
                # Point 3: Pivot


                u2 = np.array([x1, z1, y1])
                u3 = np.array([x2, z2, y2])
                u1 = np.array([x3, z3, y3])

                angle = np.degrees(np.arctan((u2[1]-u3[1])/np.sqrt(((u2[0]-u3[0])**2) + ((u2[2]-u3[2])**2))))
            
                z_offset = np.array([0, u3[1] - u1[1], 0])

                u1 += z_offset

                v1 = u2 - u3 #Pivot to Tip
                v2 = u1 - u3 #Pivot to Reference

                comp = np.dot(v2, v1)/np.linalg.norm(v2)

                # print(angle, comp)
                # print(u1, u2, u3)
                # print(v1, v2)
                # time.sleep(4)
                # print(x2 - x3, x1 - x3)
                # print(comp)


                if (comp < 0):
                    angle = angle - 90
                else:
                    angle = 90 - angle

                #angle = 90 + angle
                print(angle)


                
            


if __name__ == '__main__':
    receive_data()

