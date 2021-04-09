/*
Created on Thu April 08 10:00:22 2021

@author: MarcoRocha


To Use with BTS7960 Motor Driver
*/

#include "Arduino.h"

// Driver Library
#include "BTS7960.h"

// MOTOR COMMANDS
void startupMotor();
void enableMotor();
void disableMotor();
void stopMotor();
void setMotorSpeed(int SPEED);
