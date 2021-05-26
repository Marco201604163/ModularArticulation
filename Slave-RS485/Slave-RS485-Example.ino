/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/
// ARTICULATION PARAMS
const int myAddress = 1;
int state = 0;  // State Machine Status

// ENCODER
#include <AS5600.h>

AS5600 magnetic_encoder;

uint32_t currentMicros = 0, previousMicros = 0;
double actualPos = 0, oldPos = 0;
double angSpeed = 0, oldAngSpeed = 0;
float speedArray[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float calibOffset = 0.0;

// ARTICULATION LIBRARY
#include "ModularArticulationSens.h"
#include "ModularArticulationComs.h"
#include "ModularArticulationCtrl.h"
#include "ModularArticulationMotor.h"

// COMS FEEDBACK
const int commandStopControl = 0;
const int commandSetSpeedPos = 1;
const int commandSetSpeed = 2;
const int commandSetPos = 3;
const int commandCalibHome = 4;
const int commandGetSpeedPos = 5;
const int commandGetMotorTemp = 6;

// CYCLE VARS
uint32_t initTime = 0;
uint32_t controlTime = 0;
uint32_t controlInterval = 25UL * 1000UL; // 25 ms

float currentPos = 0.0, currentSpeed = 0.0;
int duty = 0;
int token = 0; // SIGNALS WHEN CONTROLLER ACTUATES

// Variables From Comunications
float refSpeed = 0.0;
float refPos = 0.0;


void updateJointSpeedPos(float *currentPos, float *currentSpeed, uint32_t elapsed){  
  // GET POSITION FROM MAGNETIC ENCODER
  double magnetic_encoder_raw = magnetic_encoder.getPosition();
  magnetic_encoder_raw = mapf(magnetic_encoder_raw, 0.0, 4096.0, 14.35, 4059.291);
  
  // DUE TO CALIBRATION: (360 CORRECTS ORIENTATION)
  actualPos = 360.0 - magnetic_encoder_raw * 0.089 - 1.2769;
  
  // DUE TO HOME POSITION CALIB:
  if(calibOffset != 0){
    if((actualPos >= calibOffset) and (actualPos <= 360.0)){
      actualPos = mapf(actualPos, calibOffset, 360.0, 0.0, 360.0 - calibOffset); 
    }
    else if((actualPos >= 0) and (actualPos < calibOffset)){
      actualPos = mapf(actualPos, 0.0, calibOffset, 360.0 - calibOffset, 360.0); 
    }
  }
  
  // INTERVAL: -PI TO PI
  if(actualPos > 180) actualPos = mapf(actualPos, 180, 360, -180, 0); 
  
  // VEL
  angSpeed = ((diffAngle(oldPos, actualPos) / elapsed) * 1000000UL);
  
  // ERROR VERIFICATION AND CORRECTION
  if(abs(angSpeed) > 500){
    angSpeed = oldAngSpeed;
    actualPos = oldPos + angSpeed * elapsed / 1000000UL;
  }
  
  // RUNNING AVERAGE    
  for(int i = 0; i < 4; i++){
    speedArray[i] = speedArray[i+1];
  }
  speedArray[4] = angSpeed;

  angSpeed = 0.0;
  for(int i = 0; i < 5; i++){
    angSpeed = angSpeed + speedArray[i];
  }
  angSpeed = angSpeed / 5;
  
  // VAR UPDATE
  oldPos = actualPos;
  oldAngSpeed = angSpeed;

  /*
  // INTERVAL: -PI TO PI
  if(actualPos > 180) actualPos = mapf(actualPos, 180, 360, -180, 0); 
  */
  
  // VARS TO BE CALLED IN THE MAIN LOOP
  *currentPos = actualPos;
  *currentSpeed = angSpeed; 
}

float updateJointPos(){
  // GET POSITION FROM MAGNETIC ENCODER
    delayMicroseconds(100);
  double magnetic_encoder_raw = magnetic_encoder.getPosition();
  magnetic_encoder_raw = mapf(magnetic_encoder_raw, 0, 4096, 14.35, 4059.291);
  
  // DUE TO CALIBRATION: (360 CORRECTS ORIENTATION)
  actualPos = 360.0 - magnetic_encoder_raw * 0.089 - 1.2769;
  
  return actualPos;
}

void setup()
{
  Serial.begin(9600);

  // STARTUP COMMANDS
  startArticulationComs(28800);
  startArticulationSens();
  startupMotor();
  setMotorSpeed(duty);

  // RESET TIME VARS
  initTime = micros();
  controlTime = initTime;

  // RESET ENCODER SPEED VARS
  previousMicros = micros();
  currentPos = updateJointPos();
  // INTERVAL: -PI TO PI
  if(actualPos > 180) actualPos = mapf(actualPos, 180, 360, -180, 0); 
  currentPos = actualPos;
  currentSpeed = 0;
  oldPos = actualPos;
  
  delay(2000);
}

void loop()
{
  // LISTEN AND PREPARE RESPONSE
  byte buf [16];
  byte received = 0;

  // ONLY LISTENS AFTER CONTROLLER ACTUATES
  if(token){
    received = recvMsgFromMaster(buf, sizeof(buf));
    token = 0;
  }
  // byte received = recvMsgFromMaster(buf, sizeof(buf));
  
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
        zeroControlVars();
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
      
      else if (buf [1] == 'H'){
        msg[2] = commandCalibHome; // Confirm command reception
        state = stateMachineUpdate(state, commandCalibHome); // Updates machine state;
        zeroControlVars();
      }
  
      else if (buf[1] == 'G'){
        Serial.println("GET MOTOR TEMP!");
        msg[2] = commandGetMotorTemp; // Confirm command reception
  
        // CONVERT VALUES TO BYTES
        byte tempBytes[4];
        float motorTemp = getMotorTemp();
        float2Bytes(motorTemp,&tempBytes[0]);
  
        // PUT BYTES ON MESSAGE
        for(int j = 3; j < 7; j++){
          msg[j] = tempBytes[6-j];
        }
      }
  
      else if (buf[1] == 'D'){
        Serial.println("GET POSITION AND SPEED REQUEST!");
        msg[2] = commandGetSpeedPos; // Confirm command reception
  
        // CONVERT VALUES TO BYTES
        byte actSpeedBytes[4], actPosBytes[4];
        float2Bytes(currentSpeed,&actSpeedBytes[0]);
        float2Bytes(currentPos,&actPosBytes[0]);
  
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

  // UPDATE POS, SPEED AND STATE MACHINE ACTION
  currentMicros = micros();
  uint32_t elapsed = currentMicros - previousMicros;
  
  if(elapsed >= controlInterval){
    updateJointSpeedPos(&currentPos, &currentSpeed, elapsed);

    if(state == 0){
      // STOPPED
      duty = 0;
    }else if(state == 1){
      // CASCADE CONTROLLER
      duty = runCascadeControl(rad(hermitePosReference(refPos, currentPos)), rad(currentPos), rad(currentSpeed));       
      Serial.print(refPos);
      Serial.print(" \t ");
      Serial.println(currentPos);
    }else if(state == 2){
      // PI SPEED CONTROLLER
      duty = runSpeedControl(rad(hermiteSpeedReference(refSpeed, currentSpeed)), rad(currentSpeed));
      Serial.print(refSpeed);
      Serial.print(" \t ");
      Serial.println(currentSpeed);
    }else if(state == 3){
      // PD POSITION CONTROLLER
      duty = runPosControl(rad(hermitePosReference(refPos, currentPos)), rad(currentPos));      
      Serial.print(refPos);
      Serial.print(" \t ");
      Serial.println(currentPos);
    }else if(state == 4){
      // CALIBRATE HOME POS
      int calibFlag = 0;
      calibOffset = 0.0;
      while(calibFlag < 4){ // DUE TO 4 SAMPLES
        currentPos = updateJointPos();
        calibrateHomePos(currentPos, &calibFlag, &calibOffset, &duty);
        setMotorSpeed(duty);
      }
      Serial.println(calibOffset);
      duty = 0;
      state = 0;
    }else{
      duty = 0;
      state = 0;
    }
    
    setMotorSpeed(duty);
    previousMicros = currentMicros;
    token = 1;
  }

  /*
  if ((currentMicros - initTime >= 5000000) and (currentMicros - initTime <= 7500000)){
    refPos = -150;
    refSpeed = 140;
  }
  if ((currentMicros - initTime >= 8000000) and (currentMicros - initTime <= 8500000)){
    refPos = -60;
    refSpeed = 100;
  }
  if ((currentMicros - initTime >= 11000000) and (currentMicros - initTime <= 11500000)){
    refPos = 170;
    refSpeed = 200;
  }
  */

}  // end of loop
