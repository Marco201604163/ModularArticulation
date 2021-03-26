/*
Created on Sat Mar  13 11:23:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

void startArticulationSens();

// RETURNS SENSORS VALUES
void updateJointSpeedPos(float *currentPos, float *currentSpeed);
float updateJointPos();		// (Degrees)
float updateJointSpeed();	// (Degrees/Second)
float getSupplyVoltage();	// (V)
float getMotorVoltage();	// (V)
float getMotorCurrent();	// (A)
float getMotorTemp();		// (Celsius)
int homePositionStatus();	// TRUE OR FALSE (1 OR 0)

// AUXILIARY OPERATIONS
float diffAngle(float oldAngle, float newAngle);
double mapf(double val, double inMin, double inMax, double outMin, double outMax);
float rad(float deg);
float deg(float rad);
