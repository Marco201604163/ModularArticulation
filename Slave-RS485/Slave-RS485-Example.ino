/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "ModularArticulationSens.h"
#include "ModularArticulationComs.h"
#include "ModularArticulationCtrl.h"
#include "ModularArticulationMotor.h"

const int myAddress = 1;
int state = 1;  // State Machine Status

// COMS FEEDBACK
const int commandStopControl = 0;
const int commandSetSpeedPos = 1;
const int commandSetSpeed = 2;
const int commandSetPos = 3;
const int commandSetPID = 4;
const int commandGetPID = 5;
const int commandGetSpeedPos = 6;

// CYCLE VARS
uint32_t initTime = 0;
uint32_t currentTime = 0;
uint32_t controlTime = 0;
uint32_t controlInterval = 25UL * 1000UL; // 25 ms

float currentPos = 0.0, currentSpeed = 0.0;

float duty = 0.0;

// Variables to Test Comunications
float refSpeed = 110.0;
float refPos = 60.0;
float kp = 1.23, ki = 3.45, kd = 5.67;
float actSpeed = 89.1, actPos = 183.2;

void setup()
{
  Serial.begin(9600);

  // STARTUP COMMANDS
  startArticulationComs(28800);
  delay(1);
  startArticulationSens();
  startupMotor();

  setMotorSpeed(duty);
  
  initTime = micros();
  controlTime = initTime;
  
  delay(2000);
}

void loop()
{
  // UPDATE CYCLE VARS
  updateJointSpeedPos(&currentPos, &currentSpeed);
  currentTime = micros();

  // LISTEN AND PREPARE RESPONSE
  byte buf [16];
  byte received = recvMsgFromMaster(buf, sizeof(buf));

  /*Serial.println(" ");
  Serial.println((micros() - currentTime)/1000);
  Serial.println(" ");*/
  
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

      if ((buf [1] == '-') and buf[6] == '-'){
        msg[2] = commandStopControl; // Confirm command reception
        state = stateMachineUpdate(state, commandStopControl); // Updates machine state
      }

      else if ((buf [1] == 'S') and buf[6] == 'P'){
        msg[2] = commandSetSpeedPos; // Confirm command reception
        state = stateMachineUpdate(state, commandSetSpeedPos); // Updates machine state
        refSpeed = getRefSpeedFromMessage(buf);
        refPos = getRefPosFromMessage(buf);
        zeroControlVars();
      }
  
      else if ((buf [1] == 'S') and buf[6] == '-'){
        msg[2] = commandSetSpeed; // Confirm command reception
        state = stateMachineUpdate(state, commandSetSpeed); // Updates machine state
        refSpeed = getRefSpeedFromMessage(buf);
        zeroControlVars();
      }
      
      else if ((buf [1] == '-') and buf[6] == 'P'){
        msg[2] = commandSetPos; // Confirm command reception
        state = stateMachineUpdate(state, commandSetPos); // Updates machine state
        refPos = getRefPosFromMessage(buf);
        zeroControlVars();
      }
  
      else if (buf[1] == 'C'){
        Serial.println("SET NEW PID PARAMETERS!:");
        msg[2] = commandSetPID; // Confirm command reception
        float *constants;
        constants = getNewPIDParams(buf);
        kp = *(constants);
        ki = *(constants + 1);
        kd = *(constants + 2);
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
          // ADD MORE DATA HERE IF NEEDED - JUST LIKE PID PARAMETERS
        }
      }
  
      // SEND RESPONSE MESSAGE TO MASTER
      sendMsgToMaster(msg, sizeof msg);
      // ZEROING BUF - NO MISINTERPRETATIONS
      for(int i = 0; i < sizeof(buf); i++) buf[i] = 0;
    }
   }  // END OF if(received)

    // STATE MACHINE OUTPUT ACTION
    if(state == 0){
      // STOPPED
      Serial.println("STOPPED:");
      Serial.println("STATE:");
      Serial.println(state);
    }
    else if(state == 1){
      // CASCADE MODE - RUN CASCADE CONTROLLER
      if(currentTime - controlTime >= controlInterval){
        duty = runCascadeControl(rad(hermitePosReference(refPos, currentPos)), rad(currentPos), rad(currentSpeed));
        setMotorSpeed(duty);
        controlTime = currentTime; 
      }
      Serial.print(hermitePosReference(refPos, currentPos));
      Serial.print(" \t ");
      Serial.println(currentPos);
    }
    else if(state == 2){
      // SPEED MODE - RUN PI CONTROLLER
      if(currentTime - controlTime >= controlInterval){
        duty = runSpeedControl(rad(hermitePosReference(refSpeed, currentSpeed)), rad(currentSpeed));
        setMotorSpeed(duty);
        controlTime = currentTime; 
      }
      Serial.print(hermiteSpeedReference(refSpeed, currentSpeed));
      Serial.print(" \t ");
      Serial.println(currentSpeed);
      
    }
    else if(state == 3){
      // POS MODE - RUN PD CONTROLLER
      if(currentTime - controlTime >= controlInterval){
        duty = runPosControl(rad(hermitePosReference(refPos, currentPos)), rad(currentPos));
        setMotorSpeed(duty);
        controlTime = currentTime; 
      }
      Serial.print(hermitePosReference(refPos, currentPos));
      Serial.print(" \t ");
      Serial.println(currentPos);
    }

  if ((currentTime - initTime >= 5000000) and (currentTime - initTime <= 7500000)){
    refPos = -150;
    refSpeed = 140;
  }
  if ((currentTime - initTime >= 8000000) and (currentTime - initTime <= 8500000)){
    refPos = -60;
    refSpeed = 100;
  }
  if ((currentTime - initTime >= 11000000) and (currentTime - initTime <= 11500000)){
    refPos = 170;
    refSpeed = 200;
  }

}  // end of loop
