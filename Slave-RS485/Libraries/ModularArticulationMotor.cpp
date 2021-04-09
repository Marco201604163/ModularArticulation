/*
Created on Thu April 08 10:00:22 2021

@author: MarcoRocha


To Use with BTS7960 Motor Driver
*/

#include "Arduino.h"
#include "ModularArticulationMotor.h"

// Driver Library
#include "BTS7960.h"

#define L_EN 7
#define R_EN 8
#define L_PWM 9
#define R_PWM 10

// SPEED LIMIT DUE TO VOLTAGE = (7/12) * 255
const int speedLimit = 145;

// INSTANCE INITIALIZATION
BTS7960 motor(L_EN, R_EN, L_PWM, R_PWM);

void startupMotor(){
	// Enables Motor
	enableMotor();
	
	// Sets PWM to Zero
	stopMotor();
}

void enableMotor(){
	motor.Enable();
}

void disableMotor(){
	motor.Disable();
}

void stopMotor(){
	motor.Stop();
}

void setMotorSpeed(int SPEED){
	// Checks if Speed is within Limits
	if (SPEED >= speedLimit) SPEED = speedLimit;
	else if (SPEED <= -speedLimit) SPEED = -speedLimit;
	
	// Sets Correct Command
	if (SPEED > 0) motor.TurnLeft(SPEED);
	else if (SPEED < 0) motor.TurnRight(-SPEED);
	else if (SPEED == 0) motor.Stop();
}
