/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "ModularArticulationCtrl.h"

const int commandStopCtrl = 0;
const int commandSpeedPos = 1;
const int commandSpeed = 2;
const int commandPos = 3;
const int commandPID = 4;
const int commandGetPID = 5;
const int commandGetSpeedPos = 6;

int stateMachineUpdate(const int actualState, const int commandReceived){
  int nextState = 0;
  
  switch(actualState){
    // STATE 0
    case 0 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = actualState; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = 3; break;
        case commandPos: nextState = 2; break;
      } break;
    }
    // STATE 1
    case 1 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = actualState; break;
        case commandSpeed: nextState = 3; break;
        case commandPos: nextState = 2; break;
      } break;
    }
    // STATE 2
    case 2 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = 3; break;
        case commandPos: nextState = actualState; break;
      } break;
    }
    // STATE 3
    case 3 :{
      switch(commandReceived){
        case commandStopCtrl: nextState = 0; break;
        case commandSpeedPos: nextState = 1; break;
        case commandSpeed: nextState = actualState; break;
        case commandPos: nextState = 2; break;
      } break;
    }
  }
  return nextState;
}
