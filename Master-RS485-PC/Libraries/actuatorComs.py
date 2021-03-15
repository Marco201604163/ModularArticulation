# -*- coding: utf-8 -*-
"""
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
"""

import os
import time
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import serial
import rs485
import struct

def init():
    # Opens Port
    global port
    port = serial.Serial(port="COM3", baudrate=28800)
    global rs485
    rs485 = rs485.SerialWrapper(port)
    
    
def close():
    port.close()
    
# COMMANDS FEEDBACK:
#   0 - StopControl
#   1 - SetSpeedPos
#   2 - SetSpeed
#   3 - SetPos
#   4 - SetPIDParameters
#   5 - GetPIDParameters
#   6 - GetSpeedPos
commandStopControl = 0
commandSetSpeedPos = 1
commandSetSpeed = 2
commandSetPos = 3
commandSetPID = 4
commandGetPID = 5
commandGetSpeedPos = 6

def setSpeedPos(address, speed, pos):
    message = constructControlMessage(address, speed, pos)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandSetSpeedPos)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
   
    
def setPosSpeed(address, pos, speed):
    # Call the right on
    setSpeedPos(address, speed, pos)


def setSpeed(address, speed):
    message = constructControlMessage(address, speed, '-')
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandSetSpeed)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
    
    
def setPos(address, pos):
    message = constructControlMessage(address, '-', pos)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandSetPos)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
            

def stopControl(address):
    message = constructControlMessage(address, '-', '-')
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandStopControl)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
    
    
def setPIDparameters(address, kp, ki, kd):
    message = constructSetPIDMessage(address, kp, ki, kd)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandSetPID)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
            

def getPIDparameters(address):
    message = constructGetPIDMessage(address)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
        return False
    else:
        if((response[1] == address) and (response[2] == commandGetPID)):
            [kp] = struct.unpack("!f", response[3:7])
            [ki] = struct.unpack("!f", response[7:11])
            [kd] = struct.unpack("!f", response[11:15])
            kp = round(kp, 5)
            ki = round(ki, 5)
            kd = round(kd, 5)
            print("Command correctly sent to Slave")
            print("kp: \t", kp)
            print("ki: \t", ki)
            print("kd: \t", kd)
        else:
            print("Command sent but not confirmed from Slave")
        
        return [kp, ki, kd]


def getSpeedPos(address):
    message = constructDiagMessage(address)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
        return False
    else:
        if((response[1] == address) and (response[2] == commandGetSpeedPos)):
            [actSpeed] = struct.unpack("!f", response[3:7])
            [actPos] = struct.unpack("!f", response[7:11])
            actSpeed = round(actSpeed, 5)
            actPos = round(actPos, 5)
            print("Command correctly sent to Slave")
            print("Actual Speed: \t", actSpeed)
            print("Actual Pos: \t", actPos)
        else:
            print("Command sent but not confirmed from Slave")
        
        return [actSpeed, actPos]


def listenBack():
    timeout = time.time() + 1.5 # 1.5s
    while True:
        global rs485
        if rs485.update():
            packetReceived = rs485.getPacket()
            print(len(packetReceived), " bytes received\n".encode())
            print(packetReceived)
            return packetReceived
            break
        
        elif time.time() > timeout:
            print("Timeout - No feedback from the slave")
            return False
            break


def constructControlMessage(address, refSpeed, refPos):
    # P - Position
    # S - Speed
    # '-' - No CommanD
    if ((type(refSpeed) == str) and (type(refPos) != str)): # Set Pos
        speed = 0.0
        pos = 1.0 * float(refPos)
        commandSpeed = '-'
        commandPos = 'P'
    elif ((type(refPos) == str) and type(refSpeed) != str): # Set Speed
        speed = 1.0 * float(refSpeed)
        pos = 0.0
        commandSpeed = 'S'
        commandPos = '-'
    elif ((type(refPos) == str) and type(refSpeed) == str): # Set Speed
        speed = 0.0
        pos = 0.0
        commandSpeed = '-'
        commandPos = '-'
    else: # Set Pos and Speed
        speed = 1.0 * float(refSpeed)
        pos = 1.0 * float(refPos)
        commandSpeed = 'S'
        commandPos = 'P'
    
    refSpeedInBytes = list(struct.pack("!f", speed))
    refPosInBytes = list(struct.pack("!f", pos))
    
    rawArray = [address, ord(commandSpeed), refSpeedInBytes[0], refSpeedInBytes[1], refSpeedInBytes[2], refSpeedInBytes[3], ord(commandPos), refPosInBytes[0], refPosInBytes[1], refPosInBytes[2], refPosInBytes[3]]
    package = bytearray(rawArray)
    
    return package


def constructDiagMessage(address):
    # D - Diagnose / Data Retrieval
    rawArray = [address, ord('D')]
    package = bytearray(rawArray)
    
    return package


def constructSetPIDMessage(address, kp, ki, kd):
    # C - Controller Parameters
    kp = 1.0 * float(kp)
    ki = 1.0 * float(ki)
    kd = 1.0 * float(kd)
    
    kpInBytes = list(struct.pack("!f", kp))
    kiInBytes = list(struct.pack("!f", ki))
    kdInBytes = list(struct.pack("!f", kd))
    
    
    rawArray = [address, ord('C'), kpInBytes[0], kpInBytes[1], kpInBytes[2], kpInBytes[3], kiInBytes[0], kiInBytes[1], kiInBytes[2], kiInBytes[3], kdInBytes[0], kdInBytes[1], kdInBytes[2], kdInBytes[3]]
    package = bytearray(rawArray)
    
    return package


def constructGetPIDMessage(address):
    # G - Get PID Parameters
    rawArray = [address, ord('G')]
    package = bytearray(rawArray)
    
    return package
