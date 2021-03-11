/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "RS485_protocol.h"

#include "ModularArticulationComs.h"

SoftwareSerial rs485 (2, 3);  // (ReceivePin, TransmitPin)
const byte ENABLE_PIN = 4;

void fWrite (const byte what){
	rs485.write (what);
}

int fAvailable (){
	return rs485.available ();  
}

int fRead (){
	return rs485.read ();  
}

void startArticulationComs(int br){
	rs485.begin (br);
	pinMode(ENABLE_PIN, OUTPUT);  // driver output enable
}


// MESSAGE HANDLERS
byte recvMsgFromMaster(byte * buf, const byte length){
	return recvMsg (fAvailable, fRead, buf, length);
}

void sendMsgToMaster(const byte * msg, const byte length){
	delay (1);  // give the master a moment to prepare to receive
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
    sendMsg (fWrite, msg, length);
    digitalWrite (ENABLE_PIN, LOW);  // disable sending
}


// MESSAGE "READERS"
float getRefSpeedFromMessage(byte byteArray[]){
  union {
    byte asBytes[4];
    float asFloat;
  } bytes2float;

  for(int i = 0; i < 4 ; i++){
    bytes2float.asBytes[i] = byteArray[5-i];
  }
  
  return bytes2float.asFloat;
}

float getRefPosFromMessage(byte byteArray[]){
  union {
    byte asBytes[4];
    float asFloat;
  } bytes2float;

  for(int i = 0; i < 4 ; i++){
    bytes2float.asBytes[i] = byteArray[10-i];
  }

  return bytes2float.asFloat;
}

float *getNewPIDParams(byte byteArray[] ){
  static float PIDparams[3];

  union {
    byte asBytes[4];
    float asFloat;
  } bytes2float;

  for(int j = 0; j < 3; j++){
    for(int i = 0; i < 4 ; i++){
      bytes2float.asBytes[i] = byteArray[(5-i) + (4*j)];
    }
    PIDparams[j] = bytes2float.asFloat;  
  }

  return PIDparams;
}


// DATA TYPES CONVERSIONS
int byte2int(byte input){
  return input - 48;
}

void float2Bytes(float val,byte* bytesArray){
  union {
    float floatVariable;
    byte tempArray[4];
  } float2byte;
  float2byte.floatVariable = val;
  memcpy(bytesArray, float2byte.tempArray, 4);
}
