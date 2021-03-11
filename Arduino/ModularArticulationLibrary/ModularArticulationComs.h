/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "RS485_protocol.h"


void startArticulationComs(int br);

// MESSAGE HANDLERS
byte recvMsgFromMaster(byte * buf, const byte length);
void sendMsgToMaster(const byte * data, const byte length);

// MESSAGE "READERS"
float getRefSpeedFromMessage(byte byteArray[]);
float getRefPosFromMessage(byte byteArray[]);
float *getNewPIDParams(byte byteArray[] );

// DATA TYPES CONVERSIONS
int byte2int(byte input);
void float2Bytes(float val,byte* bytesArray);