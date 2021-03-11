/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "ModularArticulationComs.h"
// #include "ModularArticulationSens.h"

int myAddress = 1;

int state = 0;
// Variables to Test Comunications
float refSpeed = 0;
float refPos = 0;
float kp = 1.23, ki = 3.45, kd = 5.67;
float actSpeed = 89.1, actPos = 183.2;

void setup()
{
  Serial.begin(9600);
  startArticulationComs(28800);
  
  Serial.println("START!");
}

void loop()
{
  byte buf [16];
  byte received = recvMsgFromMaster(buf, sizeof(buf));

  // Addresses can go from 1 to 255
  if (received)
    {
    Serial.println("RECEIVED!");
     
    if (buf [0] != myAddress)
      return;  // not my device
    else{
      Serial.println("ITS MY DEVICE!");      
      state = state + 1;
    }

    /* RESPONSE MESSAGE CONSTRUCTION TYPE:
    byte msg [] = {
       0,   // device 0 (master)
       1,   // Slave Address
       0,   // Confirm Reception
       ..., // MUST END WITH ','
    };*/

    byte msg[16];
    msg[0] = 0;
    msg[1] = myAddress;  // Slave Address

    if ((buf [1] == 'S') and buf[6] == 'P'){
      Serial.println("SET SPEED AND POS MODE!");
      msg[2] = 1; // Confirm command reception
    }

    else if ((buf [1] == 'S') and buf[6] == '-'){
      Serial.println("SET SPEED MODE!");
      msg[2] = 2; // Confirm command reception
    }
    
    else if ((buf [1] == '-') and buf[6] == 'P'){
      Serial.println("SET POS MODE!");
      msg[2] = 3; // Confirm command reception
    }

    else if (buf[1] == 'C'){
      Serial.println("SET NEW PID PARAMETERS!");
      msg[2] = 4; // Confirm command reception
    }

    else if (buf[1] == 'G'){
      Serial.println("GET PID PARAMETERS REQUEST!");
      msg[2] = 5; // Confirm command reception

      // CONVERT VALUES TO BYTES
      byte kpBytes[4], kiBytes[4], kdBytes[4];
      float2Bytes(kp,&kpBytes[0]);
      float2Bytes(ki,&kiBytes[0]);
      float2Bytes(kd,&kdBytes[0]);

      // PUT BYTES ON MESSAGE
      for(int j = 3; j < 7; j++){
        msg[j] = kpBytes[6-j];
        msg[j+4] = kiBytes[6-j];
        msg[j+8] = kdBytes[6-j];
      }
    }

    else if (buf[1] == 'D'){
      Serial.println("GET POSITION AND SPEED REQUEST!");
      msg[2] = 6; // Confirm command reception

      // CONVERT VALUES TO BYTES
      byte actSpeedBytes[4], actPosBytes[4];
      float2Bytes(actSpeed,&actSpeedBytes[0]);
      float2Bytes(actPos,&actPosBytes[0]);

      // PUT BYTES ON MESSAGE
      for(int j = 3; j < 7; j++){
        msg[j] = actSpeedBytes[6-j];
        msg[j+4] = actPosBytes[6-j];
      }
    }

    // SEND RESPONSE MESSAGE TO MASTER
    sendMsgToMaster(msg, sizeof msg);

    Serial.println("RESPONSE MESSAGE:");
    for(int i = 0; i < sizeof(msg); i++)
    {
      Serial.print(msg[i]);
      Serial.print(" ");
    }
    Serial.println(" ");

    if ((buf [1] == 'S')){
      Serial.println("SPEED RECEIVED!:");
      Serial.println(getRefSpeedFromMessage(buf));
    }

    if ((buf [6] == 'P')){
      Serial.println("POS RECEIVED!:");
      Serial.println(getRefPosFromMessage(buf));
    }

    if ((buf [1] == 'C')){
      Serial.println("CONSTANTS RECEIVED!:");
      float *constants;
      constants = getNewPIDParams(buf);
      for(int i = 0; i < 3; i ++){
        Serial.println(*(constants + i)); 
      }
    }

    // ZEROING BUF - NO MISINTERPRETATIONS
    for(int i = 0; i < sizeof(buf); i++)
    {
      buf[i] = 0;
    }

    Serial.println(" ");
    Serial.println(" ");    
    if (state > 0)
      analogWrite (11, 255);  // SET LEVEL FOR TEST LED
    else{
      analogWrite (11, 0);
     }
    
   }  

 
}  // end of loop
