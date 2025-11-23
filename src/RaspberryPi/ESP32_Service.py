from enum import Enum
import threading
from time import sleep
import time

import serial
import SerialCommunicationService
from log import *
from Buzzer import *


# from copy import deepcopy
CMD_STEER=6
CMD_SYNC=17
CMD_HEARTBEAT=7
CMD_LOG=8
CMD_SET_VMODE=9
CMD_SET_TARGETSPEED=10
CMD_SET_SMODE=18
CMD_SET_BREAKPERCENT=19
CMD_SET_TARGET_YAW=20
CMD_SET_SERVO_MAX=21
CMD_SET_SERVO_MIN=22
CMD_SET_SERVO_CENT=23
CMD_SET_UNREG=25
CMD_SET_GYRO_P=26
CMD_SET_GYRO_D=27
CMD_SET_SPCTRL_P=28
CMD_SET_SPCTRL_D=29
CMD_SET_HANDBRAKE_P=30
CMD_SET_ARCDIR=32
CMD_SET_GYRO_LATENCY_MILLIS=33
CMD_SET_SGYRO_LIMITS=34
CMD_SET_SPCTRL_I=35
CMD_SET_GYROERROR=36
CMD_SET_ARCCANCEL=37

# CMD_DATA_POSL=11
# CMD_DATA_POSR=12
# CMD_DATA_POSAVG=13
# CMD_DATA_VMODE=14
# CMD_DATA_SPEED=15
# CMD_DATA_GYRO=16
# CMD_DATA_US=24

SYNC_CODE=18

VMODE_FORWARD=1
'''vMode (velocity), sets it so the robot may move forward'''
VMODE_BACKWARD=-1
'''vMode (velocity), sets it so the robot may move backward'''
VMODE_STOP=0
'''vMode (velocity), sets it so the robot stops (no drive on wheels)'''
VMODE_BRAKE=-2
'''vMode (velocity), sets it so the robot stops by braking (counter-drive on wheels)'''
VMODE_UNREGULATED=2
'''vMode (velocity), the motors rotate with constant power without PID speed control'''
VMODE_HANDBRAKE=-3
'''vMode (velocity), the robot keeps it's current position'''
SMODE_NONE=0
'''sMode (steer), sets it so the robot does not steer'''
SMODE_GYRO=1
'''sMode (steer), sets it so the robots steering is kept straight by the gyro'''
SMODE_ARC=2
'''sMode (steer), the ESP turns until desired angle is reached'''


isSynced=False
heading=0
encoderLeft=0 #always 0 since there is no left motor
encoderRight=0 #only this encoder value is used
vMode=0
sMode=0
speed=0
logVar=0

heartbeat_thread:threading.Thread=None
ESP32LOCK=threading.Lock()
class STATE(Enum):
    HEADER=0, #Looking for packet header
    DATA=1 #Looking for data bytes
PACKET_LENGTH=27
packetBuffer=[]
serialState=STATE.HEADER
headerLetters=0
def processByte(byte):
    global serialState
    global packetBuffer
    global headerLetters
    if serialState==STATE.HEADER:
        if chr(byte)=='E':
            headerLetters=1
            # log.info("E")
        elif chr(byte)=='S' and headerLetters==1:
            headerLetters=2
            # log.info("S")
        elif chr(byte)=='P' and headerLetters==2:
            serialState=STATE.DATA
            headerLetters=0
            # log.info("P")
        # if len(packetBuffer)>5: log.debug("pb big %s"%len(packetBuffer))
        packetBuffer=[]
    elif serialState==STATE.DATA:
        packetBuffer.append(byte)
        if len(packetBuffer)==PACKET_LENGTH:
            processPacket(packetBuffer)
            # log.debug(len(packetBuffer))
            packetBuffer=[]
            serialState=STATE.HEADER
def readInt(length:int, buffer:list, delete:bool=True):
    num:int=0
    for i in range(length):
        b=int(buffer[i])
        num=(num>>(8)) | (b<<((length-1)*8))
    if num>=(2**(length*8-1)): num-=2**(length*8)
    if delete:
        del buffer[0:length]
    return num
def sendInt(data:int,data_length:int=3):
    b=bytearray(data.to_bytes(data_length,'little',signed=True))
    SerialCommunicationService.ESPserial.write(b)
packetCount=0
def processPacket(bytes:list)->bool:
    global isSynced, heading, encoderLeft, encoderRight, vMode, speed, sMode, logVar, packetCount
    packetCount+=1
    #old_bytes=deepcopy(bytes)
    checkSum=1
    isSynced_t=readInt(1,bytes)
    checkSum+=(isSynced_t%256)
    heading_t=readInt(4,bytes)
    checkSum+=(heading_t%256)
    encoderLeft_t=readInt(4,bytes)
    checkSum+=(encoderLeft_t%256)
    encoderRight_t=readInt(4,bytes)
    checkSum+=(encoderRight_t%256)
    vMode_t=readInt(1,bytes)
    checkSum+=(vMode_t%256)
    sMode_t=readInt(1,bytes)
    checkSum+=(sMode_t%256)
    speed_t=readInt(4,bytes)
    checkSum+=(speed_t%256)
    logVar_t=readInt(4,bytes)
    checkSum+=(logVar_t%256)
    
    checkSum%=256
    checkSum_t=(readInt(4,bytes)+256)%256
    if checkSum_t==checkSum:
        isSynced=isSynced_t
        heading=heading_t
        encoderLeft=encoderLeft_t
        encoderRight=encoderRight_t
        vMode=vMode_t
        
        if sMode!=sMode_t:  log.debug("smode changed from %s to %s"%(sMode,sMode_t))
        if isSynced!=2:
            log.warn(isSynced)
            # beep_parallel()
            # log.error("not in sync%s"%old_bytes)
            log.error("ESP not in sync2! logvar %s"%logVar)
        if abs(sMode)>2:    log.error("weird sMode! packet: %s %s %s %s %s %s %s %s"%(isSynced,heading, encoderLeft, encoderRight, vMode, sMode_t, sMode, logVar_t))
        sMode=sMode_t
        speed=speed_t
        logVar=logVar_t
        return True
    else:
        log.error("INCORRECT CHECKSUM! calculated %s received %s"%(checkSum,checkSum_t))
        log.warn("packet: %s %s %s %s %s %s %s %s"%(isSynced_t,heading_t,encoderLeft_t,encoderRight_t,vMode_t,sMode_t,speed_t,logVar_t))
        beep_long_parallel()
        return False
    
    # log.debug("esp packet speed %s enc1 %s"%(speed,encoderRight))
    # log.debug("received packet %s %s"%(logVar,time.time()))
    
def sendCommand(command:int, param=None):
    ESP32LOCK.acquire()
    sendInt(command,1)
    if param!=None:
        #double check!
        sendInt(param)
    ESP32LOCK.release()
def beat():
    sendCommand(CMD_HEARTBEAT)
    # log.info("speed: %s"%speed)
def getRawHeading()->int:
    return heading
def getLPos()->int:
    return encoderLeft
def getRPos()->int:
    return encoderRight
def getMotorPos()->int:
    return encoderRight
def setBrakeForce(force:int):
    sendCommand(CMD_SET_BREAKPERCENT,force)
def setSpeed(speed:int):
    sendCommand(CMD_SET_TARGETSPEED,int(speed))
def setVMode(vMode:int):
    sendCommand(CMD_SET_VMODE,vMode)
def getSMode()->int:
    return sMode
def setSteerMode(sm:int):
    sendCommand(CMD_SET_SMODE,sm)
def getSpeed()->int:
    return speed
def getVMode()->int:
    return vMode
def setServoMax(sMax):
    sendCommand(CMD_SET_SERVO_MAX,sMax)
def setServoMin(sMin):
    sendCommand(CMD_SET_SERVO_MIN,sMin)
def setServoCent(sCent):
    sendCommand(CMD_SET_SERVO_CENT,sCent)
def steer(percentage):
    sendCommand(CMD_STEER,int(percentage))
def setUnregulatedPower(power):
    sendCommand(CMD_SET_UNREG,int(power))
def setGyroPD(p,d):
    sendCommand(CMD_SET_GYRO_P,int(p))
    sendCommand(CMD_SET_GYRO_D,int(d))
def setSpeedcontrolPID(p,i,d):
    sendCommand(CMD_SET_SPCTRL_P,int(p))
    sendCommand(CMD_SET_SPCTRL_D,int(d))
    sendCommand(CMD_SET_SPCTRL_I,int(i))
def setHandbrakeP(p):
    sendCommand(CMD_SET_HANDBRAKE_P,int(p))
def handBrake():
    setVMode(VMODE_HANDBRAKE)
def setArcDir(arcDir):
    '''-1:left, 1:right'''
    sendCommand(CMD_SET_ARCDIR,arcDir)
def setGyroLatencyMillis(millis):
    sendCommand(CMD_SET_GYRO_LATENCY_MILLIS,millis)
def setSGyroLimits(limit):
    sendCommand(CMD_SET_SGYRO_LIMITS,int(limit))
def setGyroError(error):
    sendCommand(CMD_SET_GYROERROR, int(error))
    log.debug("ge: %s"%error)
def setArccancel():
    sendCommand(CMD_SET_ARCCANCEL, int(1))