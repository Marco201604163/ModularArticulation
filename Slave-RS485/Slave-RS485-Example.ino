/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "ModularArticulationComs.h"
#include "ModularArticulationCtrl.h"
// #include "ModularArticulationSens.h"

const int myAddress = 1;
int state = 0;  // State Machine Status

// COMS FEEDBACK
const int commandSetSpeedPos = 1;
const int commandSetSpeed = 2;
const int commandSetPos = 3;
const int commandSetPID = 4;
const int commandGetPID = 5;
const int commandGetSpeedPos = 6;

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

  // PROCESSES COMMANDS FROM MASTER
  if (received)
    {
    Serial.println("RECEIVED!");
    if (buf [0] != myAddress)
      return;  // not my device
    else{
      Serial.println("ITS MY DEVICE!");
      
      /* RESPONSE MESSAGE CONSTRUCTION TYPE:
      byte msg [] = {
         0,   // device 0 (master)
         1,   // Slave Address
         X,   // Confirm Reception
         ..., // MUST END WITH ','
      };*/
      
      byte msg[16];
      msg[0] = 0;
      msg[1] = myAddress;  // Slave Address

      if ((buf [1] == 'S') and buf[6] == 'P'){
        msg[2] = commandSetSpeedPos; // Confirm command reception
        state = stateMachineUpdate(state, commandSetSpeedPos); // Updates machine state
        refSpeed = getRefSpeedFromMessage(buf);
        refPos = getRefPosFromMessage(buf);
      }
  
      else if ((buf [1] == 'S') and buf[6] == '-'){
        msg[2] = commandSetSpeed; // Confirm command reception
        state = stateMachineUpdate(state, commandSetSpeed); // Updates machine state
        refSpeed = getRefSpeedFromMessage(buf);
      }
      
      else if ((buf [1] == '-') and buf[6] == 'P'){
        msg[2] = commandSetPos; // Confirm command reception
        state = stateMachineUpdate(state, commandSetPos); // Updates machine state
        refPos = getRefPosFromMessage(buf);
      }
  
      else if (buf[1] == 'C'){
        Serial.println("SET NEW PID PARAMETERS!:");
        msg[2] = commandSetPID; // Confirm command reception
        float *constants;
        constants = getNewPIDParams(buf);
        for(int i = 0; i < 3; i ++){
          Serial.println(*(constants + i)); 
        }
      }
  
      else if (buf[1] == 'G'){
        Serial.println("GET PID PARAMETERS REQUEST!");
        msg[2] = commandGetPID; // Confirm command reception
  
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
        msg[2] = commandGetSpeedPos; // Confirm command reception
  
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
      // ZEROING BUF - NO MISINTERPRETATIONS
      for(int i = 0; i < sizeof(buf); i++) buf[i] = 0;

      Serial.println("STATE:");
      Serial.println(state);
    }
   }  // END OF if(received)

    // STATE MACHINE OUTPUT ACTION
    if(state == 0){
      // STOPPED
      Serial.println("STATE:");
      Serial.println(state);
    }
    else if(state == 1){
      // SPEED AND POS MODE
      Serial.println("REF SPEED:");
      Serial.println(refSpeed);
      Serial.println("REF POS:");
      Serial.println(refPos);
      
    }
    else if(state == 2){
      // POS MODE
      Serial.println("REF POS:");
      Serial.println(refPos);
      
    }
    else if(state == 3){
      // POS MODE
      Serial.println("REF SPEED:");
      Serial.println(refSpeed);
    }

    Serial.println(" ");  
    if (state > 0)
      analogWrite (11, 255);  // SET LEVEL FOR TEST LED
    else{
      analogWrite (11, 0);
     }

}  // end of loop
