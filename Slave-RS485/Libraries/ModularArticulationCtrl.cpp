/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"

#include "ModularArticulationCtrl.h"

// CONSTANTS
#define PI 3.14159265358979

// STATE MACHINE STATES
const int commandStopCtrl = 0;
const int commandSpeedPos = 1;
const int commandSpeed = 2;
const int commandPos = 3;
const int commandPID = 4;
const int commandGetPID = 5;
const int commandGetSpeedPos = 6;

const float Ts = 0.025; // Sampling Time

// ERROR VARS - PD AND CASCADE
float posError = 0.0;
float oldPosError = 0.0;
float intPosError = 0.0;

float positionRef = 0.0;
float oldPositionRef = positionRef;


// ERROR VARS - PI SPEED
float speedError = 0.0;
float intSpeedError = 0.0;
float oldSpeedError = 0.0;

float speedRef = 0.0;
float oldSpeedRef = speedRef;

// HERMITE VARS
float pInit = positionRef, pFinal = positionRef;
float vInit = speedRef, vFinal = speedRef;
uint32_t startHermTime = 0, hermTime = millis();


int stateMachineUpdate(const int actualState, const int commandReceived){
  int nextState = 0;
  
  switch(actualState){
    // STATE 0
    case 0 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = actualState; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = 2; break;
        case commandPos: nextState = 3; break;
      } break;
    }
    // STATE 1
    case 1 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = actualState; break;
        case commandSpeed: nextState = 2; break;
        case commandPos: nextState = 3; break;
      } break;
    }
    // STATE 2
    case 2 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = actualState; break;
        case commandPos: nextState = 3; break;
      } break;
    }
    // STATE 3
    case 3 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = 2; break;
        case commandPos: nextState = actualState; break;
      } break;
    }
  }
  return nextState;
}


// CASCADE CONTROLLER
int runCascadeControl(float refPos, float actualPos, float actualSpeed){
  // CONTROL CONSTANTS
  const float kcSpeed = 15.644;
  const float Ti = 0.1564;

  const float windupLimit = 10.0;

  const float kcPos = 41.785;
  const float Td = 0.0401;

  const float minPosError = 0.01745; // 1 Deg in Rads
  // const float minPosError = 0.00875; // 0.5 Deg in Rads
  const float minSpeed = 0.687; // 39.37 deg/s in Rad
  const float offsetDZ = 15.0;  // 15 Duty Cycle

  float posOutput = 0.0;
  float output = 0.0;
  
  // Pos Error
  posError = refPos - actualPos;
  if(abs(posError - 2 * PI) < abs(posError)){
    posError = posError - 2 * PI;
  }
  if(abs(posError + 2 * PI) < abs(posError)){
    posError = posError + 2 * PI;
  }  
  
  // PD Position Loop
  posOutput = kcPos * (posError + Td * (posError - oldPosError) / Ts);

  // Speed Error
  speedError = posOutput - actualSpeed;
  intSpeedError = intSpeedError + speedError * Ts;
  
  // Anti wind up
  if(intSpeedError * (1/Ti) * kcSpeed >= windupLimit){
    intSpeedError = intSpeedError - speedError * Ts;
  }else if(intSpeedError * (1/Ti) * kcSpeed <= -windupLimit){
    intSpeedError = intSpeedError - speedError * Ts;
  }

  // PI Speed Loop
  output = kcSpeed * (speedError + (1 / Ti) * intSpeedError);

  // Dead Zone Compensation
  if((abs(posError) > minPosError) and (abs(output) < offsetDZ)){
    output = (output / abs(output)) * offsetDZ;
  }
  
  // UpdateVars
  oldPosError = posError;

  // Output = PWM Value
  return round(output);
}


// PD POSITION CONTROLLER
int runPosControl(float refPos, float actualPos) {
  // CONTROL CONSTANTS
  const float kc = 225.81;
  const float Td = 0.112;
  const float Ti = 0.10264;
  const float windupLimit = 15.0;
  
  float output = 0.0;
  
  // Error Calc
  posError = refPos - actualPos;
  if(abs(posError - 2 * PI) < abs(posError)){
    posError = posError - 2 * PI;
  }
  if(abs(posError + 2 * PI) < abs(posError)){
    posError = posError + 2 * PI;
  }
  
  // Error Integration
  intPosError = intPosError + posError * Ts;

  // Anti wind up
  if (intPosError * (1 / Ti) * kc >= windupLimit) {
    intPosError = intPosError - posError * Ts;
  }
  else if (intPosError * (1 / Ti) * kc <= -windupLimit) {
    intPosError = intPosError - posError * Ts;
  }

  // PD Loop
  // output = kc * (posError + Td * (posError - oldPosError) / Ts);
  output = kc * (posError + Td * (posError - oldPosError) / Ts + (1 / Ti) * intPosError);
  // FeedForward
  // output = output + kf * refPos;

  // UpdateVars
  oldPosError = posError;

  return round(output);
}


// PI SPEED CONTROL
int runSpeedControl(float refSpeed, float actualSpeed){
  // CONTROL CONSTANTS
  const float kc = 12.89;
  const float Ti = 0.1564;
  const float kf = 17.39;
  const float windupLimit = 10.0;

  float output = 0.0;
  // Error Calc
  speedError = refSpeed - actualSpeed;
  // Error Integration
  intSpeedError = intSpeedError + speedError * Ts;
  
  // Anti wind up
  if(intSpeedError * (1/Ti) * kc >= windupLimit){
    intSpeedError = intSpeedError - speedError * Ts;
  }
  else if(intSpeedError * (1/Ti) * kc <= -windupLimit){
    intSpeedError = intSpeedError - speedError * Ts;
  }

  // PD Loop
  output = kc * (speedError + (1 / Ti) * intSpeedError);
  // FeedForward
  output = output + kf * refSpeed;

  // Output = PWM Value
  return round(output);
}

// GENERATE HERMITE POLYNOMIALS - POSITION
float hermitePosReference(float refPos, float currentPos){
  // UPDATES TIME AND VARS - ms
  positionRef = refPos;
  hermTime = millis() - startHermTime;
  
  // CHECKS IF IT IS A NEW REFERENCE
  if(positionRef != oldPositionRef){
    // pInit = oldPositionRef;
    pInit = currentPos;
    pFinal = positionRef;
    oldPositionRef = positionRef;
    startHermTime = millis();
    hermTime = 0;    
  }
  
  // CHECKS IF HERM INTERVAL HAS PASSED
  float timeScale = 0.0;
  if (pInit != pFinal) timeScale =  180.0 / abs(pInit - pFinal);
  else timeScale = 1;
  
  if(hermTime * timeScale >= 1000) return pFinal;
  else{
    float t = float(hermTime) * timeScale / 1000.0;
    return (2*pow(t, 3) - 3*pow(t, 2) + 1) * pInit + (-2*pow(t, 3) + 3*pow(t, 2)) * pFinal;
  }
}


// GENERATE HERMITE POLYNOMIALS - SPEED
float hermiteSpeedReference(float refSpeed, float currentSpeed){
  // UPDATES TIME - ms
  speedRef = refSpeed;
  hermTime = millis() - startHermTime;
  
  // CHECKS IF IT IS A NEW REFERENCE
  if(speedRef != oldSpeedRef){
    //vInit = oldSpeedRef;
    vInit = currentSpeed;
    vFinal = speedRef;
    oldSpeedRef = speedRef;
    startHermTime = millis();
    hermTime = 0;    
  }
  
  // CHECKS IF HERM INTERVAL HAS PASSED
  float timeScale = 0.0;
  if (vInit != vFinal) timeScale =  90.0 / abs(vInit - vFinal);
  else timeScale = 1;
  
  if(hermTime * timeScale >= 1000) return vFinal;
  else{
    float t = float(hermTime) * timeScale / 1000.0;
    return (2*pow(t, 3) - 3*pow(t, 2) + 1) * vInit + (-2*pow(t, 3) + 3*pow(t, 2)) * vFinal;
  }
}


// RESET CONTROL VARS
void zeroControlVars(){
  // ZEROING CASCADE VARS
  speedError = 0.0;
  intSpeedError = 0.0;
  oldSpeedError = 0.0;

  posError = 0.0;
  oldPosError = 0.0;
  intPosError = 0.0;
  
  positionRef = 0;
  oldPositionRef = positionRef;
  pInit = positionRef, pFinal = positionRef;
  
  speedRef = 0.0;
  oldSpeedRef = speedRef;
  vInit = speedRef, vFinal = speedRef;
	
  startHermTime = 0, hermTime = millis();
}
