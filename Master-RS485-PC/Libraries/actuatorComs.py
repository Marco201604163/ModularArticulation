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

def init(USBport):
    # Opens Port
    global port
    port = serial.Serial(port=USBport, baudrate=28800)
    global rs485
    rs485 = rs485.SerialWrapper(port)
    
    
def close():
    port.close()
    
# COMMANDS FEEDBACK:
#   0 - StopControl
#   1 - SetSpeedPos
#   2 - SetSpeed
#   3 - SetPos
#   4 - CalibrateHomePositionSensor
#   5 - SetPIDParameters
#   6 - GetPIDParameters
#   7 - GetSpeedPos
commandStopControl = 0
commandSetSpeedPos = 1
commandSetSpeed = 2
commandSetPos = 3
commandCalibHomeSensor = 4
commandGetSpeedPos = 5
commandGetTemp = 6

def setCascade(address, pos):
    speed = 0
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
       

def calibHomePosSensor(address):
    message = constructCalibMessage(address)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandCalibHomeSensor)):
            print("Command correctly sent to Slave")
        else:
            print("Command sent but not confirmed from Slave")
    
    
def getMotorTemp(address):
    message = constructGetTempMessage(address)
    global rs485
    rs485.sendMsg(message)
    time.sleep(0.1)
    response = listenBack()
    if(response == False):
        print("No Response from Slave")
    else:
        if((response[1] == address) and (response[2] == commandGetTemp)):
            [motorTemp] = struct.unpack("!f", response[3:7])
            motorTemp = round(motorTemp, 5)
            print("Command correctly sent to Slave")
            print("Motor Temp: \t", motorTemp)
        else:
            print("Command sent but not confirmed from Slave")
            
        return [motorTemp]
        

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


def constructCalibMessage(address):
    # D - Diagnose / Data Retrieval
    rawArray = [address, ord('H')]
    package = bytearray(rawArray)
    
    return package


def constructGetTempMessage(address):
    # G - Get PID Parameters
    rawArray = [address, ord('G')]
    package = bytearray(rawArray)
    
    return package
