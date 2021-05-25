/*
Created on Tue Mar  9 15:53:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"

// CONTROL RELATED FUNCTIONS
int stateMachineUpdate(const int actualState, const int commandReceived);
void zeroControlVars();

// CONTROLLERS - SPEED AND POS
int runCascadeControl(float refPos, float actualPos, float actualSpeed);
int runPosControl(float refPos, float actualPos);
int runSpeedControl(float refSpeed, float actualSpeed);

// GENERATE HERMITE POLYNOMIALS AS REFERENCES
float hermitePosReference(float refPos, float currentPos);
float hermiteSpeedReference(float refSpeed, float currentSpeed);
