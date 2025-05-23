from enum import Enum
from time import sleep, time
from log import *

import serial

newLidarDataFlag=False
onLidarRev=None
onLoop=None
DISTANCE_MAP:list[float]=[-1]*360
INTENSITY_MAP:list[int]=[0]*360

class STATE(Enum):
    HEADER=0,
    VERLEN=1,
    DATA=2
getMotorPosLidar=None
act_state=STATE.HEADER
errors=0
PACKET_SIZE=47
byte_packet=[0]*PACKET_SIZE
lidarCount:int=0
t0=time()
lastAngle=0
lookingForLoop=False
log.debug("imported")
def process_packet(packet_in:list):
    global DISTANCE_MAP, newLidarDataFlag, INTENSITY_MAP, lookingForLoop, lastAngle
    start_angle=packet_in[5]*256+packet_in[4]
    end_angle=packet_in[43]*256+packet_in[42]
    step:float=(end_angle-start_angle+360*1000)%36000/100/(12-1)
    for i in range(12):

        distance:float=(packet_in[7+i*3]*256+packet_in[6+i*3])/10
        intensity:int=packet_in[8+i*3]
        angle:int=round(start_angle/100+step*i)%360
        DISTANCE_MAP[angle]=distance
        INTENSITY_MAP[angle]=intensity
        if ((angle-lastAngle)+3600)%360>1:
            # log.debug("filling missing degrees %s %s"%(angle,lastAngle))
            for i in range(lastAngle+1,angle):
                DISTANCE_MAP[i]=distance #fill missing degrees
                INTENSITY_MAP[i]=intensity
        lastAngle=angle
    
    
    if lookingForLoop and (end_angle<10000): #looped
        lookingForLoop=False
        newLidarDataFlag=True
        # log.debug("l %s %s"%(DISTANCE_MAP[270],getMotorPosLidar()))
    if end_angle>=18000 and lookingForLoop==False:
        # log.debug("lookingforloop %s"%lookingForLoop)
        lookingForLoop=True
    
def processByte(byte:int):
    global lidarCount
    global act_state
    global errors
    if act_state==STATE.HEADER:
        if byte==0x54:
            byte_packet[lidarCount]=byte
            lidarCount+=1
            act_state=STATE.VERLEN
        else:
            errors+=1
    elif act_state==STATE.VERLEN:
        if byte==0x2c:
            byte_packet[lidarCount]=byte
            lidarCount+=1
            act_state=STATE.DATA
        else:
            lidarCount=0
            act_state=STATE.HEADER
    elif act_state==STATE.DATA:
        byte_packet[lidarCount]=byte
        lidarCount+=1
        if lidarCount>=PACKET_SIZE:
            act_state=STATE.HEADER
            lidarCount=0
            process_packet(byte_packet)
LIDAR_DEADZONE_START=-130
LIDAR_DEADZONE_END=130
