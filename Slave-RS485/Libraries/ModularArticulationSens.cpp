/*
Created on Sat Mar  13 11:23:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"

// TEMPERATURE
#include <OneWire.h>
#include <DallasTemperature.h>

// ENCODER
#include <AS5600.h>

#include "ModularArticulationSens.h"

// ANALOG INPUTS
#define motorCurrentSensor A0
#define supplyVoltageSensor A1	// ONLY POSITIVE VOLTAGES
#define motorVoltageSensor A2	// POSITIVE AND NEGATIVE VOLTAGES

// CONSTANTS
#define PI 3.14159265358979

// DIGITAL INPUTS
#define motorTempSensor 7 // D7

// TEMPERATURE - INITIALIZATION
OneWire oneWire(motorTempSensor);	// One Wire Comunication
DallasTemperature tempSensor(&oneWire);

// ENCODER - INITIALIZATION
AS5600 magnetic_encoder;

uint32_t interval = 75UL * 1000UL; // 75ms
uint32_t currentMicros, previousMicros;
double angSpeed = 0, actualPos = 0, oldPos = 0;


void startArticulationSens(){
	tempSensor.begin();				// Starts One-Wire Comunication
	tempSensor.setResolution(9);	// Less Resolution = Faster Conversion
	// IMPORTANT NOTE: If temperature reaches ridiculous values,
	//		must verify function above
	// CONSULT: https://lastminuteengineers.com/ds18b20-arduino-tutorial/
	tempSensor.setWaitForConversion(false);
	
	
	// ANALOG INPUTS - VOLTAGE AND CURRENT
	pinMode(motorCurrentSensor, INPUT);
	pinMode(supplyVoltageSensor, INPUT);
	pinMode(motorVoltageSensor, INPUT);
	
	// VARIABLES INIT - SPEED
	previousMicros = micros();
}

float updateJointPos(){
	// GET POSITION FROM MAGNETIC ENCODER
    delayMicroseconds(100);
	double magnetic_encoder_raw = magnetic_encoder.getPosition();
	magnetic_encoder_raw = mapf(magnetic_encoder_raw, 0, 4096, 14, 4059);
	
	// DUE TO CALIBRATION:
	oldPos = actualPos;
	actualPos = magnetic_encoder_raw * 0.089 - 1.2769;
	
	return actualPos;
}

float updateJointSpeed(){
	// CALCULATES JOINT SPEED AFTER POSITION UPDATE
	currentMicros = micros();
	uint32_t elapsed = currentMicros - previousMicros;
	
	if(elapsed >= interval){
		// SPEED CALC
		previousMicros = currentMicros;
		// Degrees per Second
		angSpeed = ((diffAngle(oldPos, actualPos) / elapsed) * 1000000UL);
    }
	return angSpeed;
}

void updateJointSpeedPos(float *currentPos, float *currentSpeed){
	// FUNCTION TO BE CALLED IN THE MAIN LOOP
	*currentPos = updateJointPos();
	*currentSpeed = updateJointSpeed();	
}

float getSupplyVoltage(){
	// ONLY POSITIVE VOLTAGES
	float average = 0.0, v = 0.0;
	int i = 0;
	
    while(i < 10){
		v = analogRead(supplyVoltageSensor);
		delay(1);
		average = (average * i + v) / (i + 1);
		i = i + 1;
    }
	return (0.0143 * average + 0.0234); // V
}

float getMotorVoltage(){
	// POSITIVE ANG NEGATIVE VOLTAGES
	float average = 0.0, v = 0.0;
	int i = 0;
	
    while(i < 10){
		v = analogRead(motorVoltageSensor);
		delay(1);
		average = (average * i + v) / (i + 1);
		i = i + 1;
    }
	return (0.0334 * average - 19.352); // V
}

float getMotorCurrent(){
	// POSITIVE CURRENTS ONLY
	float average = 0.0, amps = 0.0;
	int i = 0;
	
    while(i < 10){
		amps = analogRead(motorCurrentSensor);
		delay(1);
		average = (average * i + amps) / (i + 1);
		i = i + 1;
    }
	return (0.0046 * average + 0.0283); // Amps
}

float getMotorTemp(){
	// USES DALLAS (MANUFACTURER) LIBRARY
	tempSensor.requestTemperatures();
	delay(1);
	return (tempSensor.getTempCByIndex(0));
}

float diffAngle(float oldAngle, float newAngle){
  // KEEPS DIFF BETWEEN -180 AND +180 DEGREES
  float diff = newAngle - oldAngle;
  
  if (diff > 180.0) diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  
  return diff;
}

double mapf(double val, double inMin, double inMax, double outMin, double outMax){
   return ((val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

float rad(float deg){
	return (deg * PI / 360);
}

float deg(float rad){
	return (rad * 360 / PI);
}





