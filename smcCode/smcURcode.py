import socket
import time
import struct
import math
import numpy as np
import csv

global V

def sign(v):
    if v > 0:
        return 1
    elif v < 0:
        return -1
    elif v == 0:
        return 0
    else:
        return v

def connect(HOST, PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    return s


def smc( des_pos, pos, vel):

    lmd = 7.2
    rho = 4
    e_pos = [0, 0, 0]
    ut = [0, 0, 0]
    e_vel = [0, 0, 0]

    e_pos[0] = des_pos[0] - pos[0]
    e_pos[1] = des_pos[1] - pos[1]
    e_pos[2] = des_pos[2] - pos[2]
    e_vel[0] = 0.008
    e_vel[1] = 0.008
    e_vel[2] = 0.008

    sx = -e_vel[0] + lmd * e_pos[0]
    sy = -e_vel[1] + lmd * e_pos[1]
    sz = -e_vel[2] + lmd * e_pos[2]
        
    ut[0] = float(6 * e_vel[0] + rho * sign(sx))
    ut[1] = float(6 * e_vel[1] + rho * sign(sy))
    ut[2] = float(6 * e_vel[2] + rho * sign(sz))

    return ut

def pCont( des_pos, pos):
    ut = [0, 0, 0]
    ut[0] = float(50*(des_pos[0] - pos[0]))
    ut[1] = float(50*(des_pos[1] - pos[1]))
    ut[2] = float(50*(des_pos[2] - pos[2]))
    return ut

def getCurrpos(s):
    s.recv(444)
    tcp_positionx = s.recv(8)
    x = struct.unpack('!d', tcp_positionx)[0]
    tcp_positiony = s.recv(8)
    y = struct.unpack('!d', tcp_positiony)[0]
    tcp_positionz = s.recv(8)
    z = struct.unpack('!d', tcp_positionz)[0]
    tcp_positionrx = s.recv(8)
    rx = struct.unpack('!d', tcp_positionrx)[0]
    tcp_positionry = s.recv(8)
    ry = struct.unpack('!d', tcp_positionry)[0]
    tcp_positionrz = s.recv(8)
    rz = struct.unpack('!d', tcp_positionrz)[0]
    curr_pos = [x, y, z, rx, ry, rz]
    s.recv(568)
    return curr_pos

def getXdot(s):
    import struct
    s.recv(492)
    xdotdata = s.recv(8)
    xdot = struct.unpack('!d', xdotdata)[0]
    ydotdata = s.recv(8)
    ydot = struct.unpack('!d', ydotdata)[0]
    zdotdata = s.recv(8)
    zdot = struct.unpack('!d', zdotdata)[0]
    rxdotdata = s.recv(8)
    rxdot = struct.unpack('!d', rxdotdata)[0]
    rydotdata = s.recv(8)
    rydot = struct.unpack('!d', rydotdata)[0]
    rzdotdata = s.recv(8)
    rzdot = struct.unpack('!d', rzdotdata)[0]
    s.recv(520)
    tcp_vel = [xdot, ydot, zdot, rxdot, rydot, rzdot]
    return tcp_vel


HOST = "192.168.0.9"    # The remote host
PORT = 30003              # The same port as used by the server
print ("Starting Program")
count = 0
# while (count == 0):
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.connect((HOST, PORT))
#s.listen(5) # Now wait for client connection.
print ("connected")
dx = -0.06941
# dx = 0.23177023343745867
dy = -0.58648
dz = 0.45457
# dz = 0.730
rx = -0.74669
ry = 2.89317
rz = -0.23178
des_pos = [dx, dy, dz, rx, ry, rz]

time.sleep(2)
msg = 'movej(p['+str(des_pos[0])+','+str(des_pos[1])+','+str(des_pos[2])+','+ '-0.74669'+\
','+'2.89317'+','+'-0.23178'+'],a='+'1.3962634015954636'+',v='+'1.0471975511965976'+')'
msg = msg.encode()
ter = '\n'.encode()
s.send(msg + ter)
time.sleep(2)
theta = (2 * math.pi)/10

for i in np.arange(0, 11, 1):
    dx = -0.1941 + (0.250 * math.cos(theta + (i*theta))) #circle
    dy = -0.40648 + (0.250 * math.sin(theta + (i*theta)))

    des_pos = [dx, dy, dz, rx, ry, rz]

    for t in np.arange(0, 80, (1/125)):

        s = connect(HOST, PORT)
        curr_pos = getCurrpos(s)
        curr_vel = getXdot(s)

        u = smc(des_pos, curr_pos, curr_vel)

        curr_pos[0] = curr_pos[0] + t*u[0]
        curr_pos[1] = curr_pos[1] + t*u[1]
        curr_pos[2] = curr_pos[2] + t*u[2]


        msg1 = 'movej(p['+str(curr_pos[0])+','+str(curr_pos[1])+','+str(curr_pos[2])+','+ str(curr_pos[3])+\
        ','+str(curr_pos[4])+','+str(curr_pos[5])+'],a='+'1.3962634015954636'+',v='+'1.0471975511965976'+')'
        msg1 = msg1.encode()
        ter = '\n'.encode()
        s.send(msg1 + ter)
        time.sleep(3)

        time.sleep(2)
        s.close()

        if abs(abs(curr_pos[0]) - abs(des_pos[0])) < 0.04 or abs(abs(curr_pos[1]) - abs(des_pos[1])) < 0.04\
             or abs(abs(curr_pos[2]) - abs(des_pos[2])) < 0.04:
            for t in np.arange(0, 80, (1/125)):
                s = connect(HOST, PORT)

                curr_pos = getCurrpos(s)

                u = pCont(des_pos, curr_pos)

                curr_pos[0] = curr_pos[0] + t*u[0]
                curr_pos[1] = curr_pos[1] + t*u[1]
                curr_pos[2] = curr_pos[2] + t*u[2]
                # print(curr_pos[0],", ", curr_pos[1], ", ", curr_pos[2])
                

                msg1 = 'movej(p['+str(curr_pos[0])+','+str(curr_pos[1])+','+str(curr_pos[2])+','+ str(curr_pos[3])+\
                ','+str(curr_pos[4])+','+str(curr_pos[5])+'],a='+'1.3962634015954636'+',v='+'1.0471975511965976'+')'
                msg1 = msg1.encode()
                ter = '\n'.encode()
                s.send(msg1 + ter)
                time.sleep(1)
                s.close()

                if abs(abs(curr_pos[0]) - abs(des_pos[0])) < 0.005 and abs(abs(curr_pos[1]) - abs(des_pos[1])) < 0.005\
                    and abs(abs(curr_pos[2]) - abs(des_pos[2])) < 0.005:
                    # print(curr_pos[0],", ", curr_pos[1], ", ", curr_pos[2])
                    break
            break

    # i = i + 1

print ("Program finish")
