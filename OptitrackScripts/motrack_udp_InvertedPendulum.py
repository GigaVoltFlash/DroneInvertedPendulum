    ###########################################################################
#   motrack_udp_multi.py
#
#   v1.0.0
#   written by Miles Johnson, Aaron Phelps, and Cem Onglsku
#
#   reads 6-DOF on multiple trackables from a running Tracking Tools
#   (go to Streaming Pane and check the box for 'Broadcast Frame Data'
#   in NaturalPoint Streaming Engine, For "Network Interface Selection",
#   under "Local Interface" set to <opti_ip> ) and sends the data to
#   <drone_ip>, a list of IPs for laptops running drones.
#
#    Note that the order of IP addresses in drone_ip corresponds to the
#    trackable numbers.  These must match!
#
#    Copies of this code are found on the svn at
# https://subversion.cs.illinois.edu/svn/ae483/trunk/optitrack/motrack_udp_multi.py
# -------------------------------------------------------------------------
#   Change Log:
#
#   
###########################################################################

import sys
import socket
import struct
import threading
import time
import math

import serial
import Queue
import fmcore
import scipy.io
from policies import lqr_outer

from utils import *


##################################################################
######## Configuration Options ###################################
##################################################################

# Drone Computer IP's - First IP responds to first trackable, etc.
# To increase number of drones, add IP's (run "ifconfig eth0" on Linux machines)
#drone_ip = ["192.168.0.109"]
#drone_ip = "192.168.1.51", "192.168.1.52","192.168.1.54"
# drone_ip = "192.168.0.51", "192.168.0.52","192.168.0.54"
#drone_ip = ["192.168.1.72"]

# Drone Computer UDP Port
#drone_port = 3500
udp_port = 3500
# address_list = ["192.168.1.166"]
address_list = ["192.168.1.103"]

            
# OptiTrack Computer IP address
# [Start->type 'cmd' in command window, type IPconfig, IPv4]
# opti_ip ="192.168.1.99"
opti_ip ="192.168.0.99"

# Data Port Set in Optitrack Streaming Properties
opti_port = 1511

# Multicast Interface in Optitrack Streaming Properties
multicastAdd = "239.255.42.99"

##################################################################
##################################################################
# DO NOT EDIT ANYTHING BENEATH THIS LINE!!!!
##################################################################
##################################################################

def unPack(data):
    trackableState = []
    markerState = []
    byteorder='@'
    PacketIn = data
    major = 2
    minor = 0
    offset = 0
    # message ID, nBytes
    messageID, nBytes = struct.unpack(byteorder+'hh',PacketIn[offset:offset+4])
    offset += 4
    #print 'messageID=',messageID,' number of bytes=',nBytes
    if (messageID == 7):
        frameNumber,nMarkerSets = struct.unpack(byteorder+'ii',PacketIn[offset:offset+8])
        offset += 8
        #print 'Markersets=', nMarkerSets
        i=nMarkerSets
        while (i > 0):
            ns = PacketIn[offset:offset+255]
            szNamelen=ns.find('\0')
            #print ns, szNamelen
            szName = struct.unpack(byteorder+str(szNamelen)+'s',PacketIn[offset:offset+szNamelen])[0]
            offset += szNamelen+1 # include the C zero char
            #print 'Modelname=',szName
            # markers
            nMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Markercount=',nMarkers
            j=nMarkers
            while (j>0):
                x,y,z = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                offset += 12
                j=j-1
            i=i-1

        #unidentified markers
        nUMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'Unidentified Markercount=',nUMarkers
        i = nUMarkers
        while (i > 0):
            ux,uy,uz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            markerState.append([ux,uy,uz])
            #print [ux,uy,uz]
            i=i-1

        # rigid bodies
        nrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        nr = nrigidBodies
        #print nr
        offset += 4
        #print 'Rigid bodies=',nrigidBodies
        while (nr > 0):
            ID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            rbx,rby,rbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            rbqx,rbqy,rbqz,rbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
            offset += 16

            trackableState.append([ID,frameNumber, rbx, rby, rbz, rbqw, rbqx, rbqy, rbqz])  # our quaternion convention is scalar part first!

            #print '\nID=',ID
            #print 'pos:',rbx,rby,rbz
            #print 'ori:',rbqx,rbqy,rbqz,rbqw
            # associated marker positions
            nRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Marker count=',nRigidMarkers
            md = []
            markerID = []
            markersize = []
            for i in range(0,nRigidMarkers):
                md.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                offset += 12
            if major >= 2:
                for i in range(0,nRigidMarkers):
                    markerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    markersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' ID=',markerID[i],' markerData=',md[3*i],md[3*i+1],md[3*i+2],' markersize=',markersize[i]
            else:
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' markerData=',md[3*i],md[3*i+1],md[3*i+2]

            # marker errors
            if major >= 2:
                markerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'Mean marker error=',markerError

            nr = nr-1 # next rigid body

        #skeletons
        if (major==2 and minor>0) or major>2:
            nSkeletons = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Skeletons=',nSkeletons
            ns = nSkeletons
            while (ns > 0):
                skeletonID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'SkeletonID=',skeletonID
                nsrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                nsr = nsrigidBodies
                offset += 4
                #print 'Rigid body count=',nsrigidBodies
                while (nsr > 0):
                    IDsr = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    srbx,srby,srbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                    offset += 12
                    srbqx,srbqy,srbqz,srbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
                    offset += 16
                    #print 'ID=',IDsr
                    #print 'pos:',srbx,srby,srbz
                    #print 'ori:',srbqx,srbqy,srbqz,srbqw
                    # associated marker positions
                    nsRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Marker count=',nsRigidMarkers
                    smd = []
                    smarkerID = []
                    smarkersize = []
                    for i in range(0,nsRigidMarkers):
                        smd.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                        offset += 12
                    for i in range(0,nsRigidMarkers):
                        smarkerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        smarkersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        pass
                        #print 'Marker ',i+1,' ID=',smarkerID[i],' markerData=',smd[3*i],smd[3*i+1],smd[3*i+2],' markersize=',markersize[i]

                    # marker errors
                    smarkerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Mean marker error=',smarkerError

                    nsr = nsr-1 # next rigidbody of skeleton

                ns = ns-1 # next skeleton

        #latency
        latency = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'latency=',latency
        #end of data tag
        eod = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'end of packet'
    
    return trackableState, markerState


if __name__ == '__main__': #{{{1

    # Initialize Multicast Socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    #s.bind((opti_ip, opti_port))
    s.bind(('', opti_port)) # check if this makes a difference
    mreq = struct.pack('4sl',socket.inet_aton(multicastAdd), socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
	
    # Initialize UDP Socket
    udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    # Create Drone Sockets - DONT NEED THIS ANYMORE, replaced by address list
    #clientlist = []
    #for x in drone_ip:
    #    client = FMClientUDP(address=(x,drone_port))
    #    clientlist.append(client)
    #    client.start()

    # Receive data
	
    # Create data arrays
    mocap_state = [] 
    prev_state = []
    status = []
    rbid = [] # rigid body id
    pymsg = [] 
    lastState = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0,0.0,0.0]]
    print "begin recv'ing"
	
    frame_counter = 1.
	
    while (True):
		
        #data, addr = s.recvfrom(20240) #receive data and address from Optitrack Multicast
        data = s.recv(10240) #receive data and address from Optitrack Multicast
		
        if data:
			
            # Unpack data and skip if data isn't valid
            state, markerState = unPack(data)
            print markerState
            if (len(markerState) < 3):
                data = struct.pack('fffffffff', lastState[0][0], lastState[0][1], lastState[0][2],lastState[1][0], lastState[1][1], lastState[1][2], lastState[2][0], lastState[2][1], lastState[2][2])
                udpsock.sendto(data, (address_list[0],3500))
            else:
                data = struct.pack('fffffffff', markerState[0][0], markerState[0][1], markerState[0][2],markerState[1][0], markerState[1][1], markerState[1][2],markerState[2][0], markerState[2][1], markerState[2][2])
                lastState = markerState
                udpsock.sendto(data, (address_list[0],3500))

    


            

