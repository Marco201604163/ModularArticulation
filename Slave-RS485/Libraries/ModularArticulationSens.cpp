/*
Created on Sat Mar  13 11:23:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"

// TEMPERATURE
#include <OneWire.h>
#include <DallasTemperature.h>

#include "ModularArticulationSens.h"

// ANALOG INPUTS
#define supplyVoltageSensor A0	// ONLY POSITIVE VOLTAGES
#define motorCurrentSensorINA169 A1	// POSITIVE AND NEGATIVE VOLTAGES
#define motorCurrentSensorR A2	// MOTOR R SIDE CURRENT
#define motorCurrentSensorL A3	// MOTOR L SIDE CURRENT

// CONSTANTS
#define PI 3.14159265358979

// DIGITAL INPUTS
#define motorTempSensor 11 // D11
#define homePosSensor 12 // D12 

// TEMPERATURE - INITIALIZATION
OneWire oneWire(motorTempSensor);	// One Wire Comunication
DallasTemperature tempSensor(&oneWire);
DeviceAddress tempSensorAddress;

// HOME POSITION SENSOR VARS
int hallStatus = 0, oldHallStatus = 0;

void startArticulationSens(){
	tempSensor.begin();								// Starts One-Wire Comunication
	tempSensor.getAddress(tempSensorAddress, 0);	// Get Temp Sensor Address
	tempSensor.setResolution(tempSensorAddress, 9);	// Less Resolution = Faster Conversion
	// IMPORTANT NOTE: If temperature reaches ridiculous values,
	//		must verify function above
	// CONSULT: https://lastminuteengineers.com/ds18b20-arduino-tutorial/
	tempSensor.setWaitForConversion(false);
	tempSensor.setHighAlarmTemp(tempSensorAddress, 125); // Set to Max Temp
	
	
	// ANALOG INPUTS - VOLTAGE AND CURRENT
	pinMode(supplyVoltageSensor, INPUT);
	pinMode(motorCurrentSensorINA169, INPUT);
	pinMode(motorCurrentSensorR, INPUT);
	pinMode(motorCurrentSensorL, INPUT);
	
	
	// DIGITAL INPUTS - HOME POSITION
	pinMode(homePosSensor, INPUT);

	// SPEEDS UP I2C COMS
	TWBR = 1; 
	
	// RESETS HALL STATUS VARS
	hallStatus = homePositionStatus();
	oldHallStatus = hallStatus;
}

float getSupplyVoltage(){
	// ONLY POSITIVE VOLTAGES
	float average = 0.0, v = 0.0;
	int i = 0;
	
    while(i < 5){
		v = analogRead(supplyVoltageSensor);
		delayMicroseconds(100);
		average = (average * i + v) / (i + 1);
		i = i + 1;
    }
	return (0.0143 * average + 0.0034); // V
}

float getMotorVoltage(int dutyPWM){
	// POSITIVE ANG NEGATIVE VOLTAGES
	return (dutyPWM / 255.0) * getSupplyVoltage();
}

float getMotorCurrent(){
	// POSITIVE CURRENTS ONLY
	float average = 0.0, amps = 0.0, ampsR = 0.0, ampsL = 0.0;
	float Ris = 900; // Adjust to new value if Ris changed
	int i = 0;
	
    while(i < 5){
		ampsR = analogRead(motorCurrentSensorR);
		delayMicroseconds(100);
		ampsL = analogRead(motorCurrentSensorL);
		// ADJUST SIGN ACCORDING TO INSTALLATION
		amps = ampsR + ampsL;
		delayMicroseconds(100);
		average = (average * i + amps) / (i + 1);
		i = i + 1;
    }
	
	average = (average / 1023.0) * 5.0;
	
	return (average * 8500.0 / Ris); // Amps
}

float getMotorCurrentINA169(){
	// POSITIVE CURRENTS ONLY
	const float RS = 0.1, VOLTAGE_REF = 5.0;
	float average = 0.0, amps = 0.0;
	double currentValue = 0.0;
	int i = 0;

	// AVERAGE
	while(i < 10){
      amps = analogRead(motorCurrentSensorINA169);
      delayMicroseconds(100);
      average = (average * i + amps) / (i + 1);
      i = i + 1;
    }
    currentValue = 0.0046 * average + 0.0283;
	
	return currentValue; // Amps
}

float getMotorTemp(){
	// USES DALLAS (MANUFACTURER) LIBRARY
	tempSensor.requestTemperatures();
	delay(1);
	return (tempSensor.getTempC(tempSensorAddress));
}

void setMotorTempAlarm(int maxTemp){
	// SETS MAX ALLOWED TEMPERATURE
	tempSensor.setHighAlarmTemp(tempSensorAddress, maxTemp);
}

int getMotorTempAlarmStatus(){
	// CHECKS TEMPERATURE ALARM FLAG
	return tempSensor.hasAlarm(tempSensorAddress);
}

void stopMotorTempAlarm(){
	// TURNS OFF ALARM
	tempSensor.setHighAlarmTemp(tempSensorAddress, 125);
}

int homePositionStatus(){
	// RETURNS 1 IF THE HOME POSITION IT'S REACHED
	// Digital pin with 10k Ohm pull-up resistor
	return(not(digitalRead(homePosSensor)));
}

// CALIBRATES HOME POSITION - RETURNS 1 SUCESSFULLY
void calibrateHomePos(float currentPos, int *flag, float *calibOffset, int *duty){
	hallStatus = homePositionStatus();
	
	// RE
	if((hallStatus == 1) and (oldHallStatus == 0)){
		*calibOffset = (*calibOffset * (*flag) + currentPos) / (*flag + 1);
		*flag += 1;
	}
	
	// FE
	if((hallStatus == 0) and (oldHallStatus == 1)){
		*calibOffset = (*calibOffset * (*flag) + currentPos) / (*flag + 1);
		*flag += 1;
	}	
	
	// UPDATES VARS
	oldHallStatus = hallStatus;
	
	// HANDLES DUTY
	if(*flag < 2) *duty =75;
	else *duty = -75;
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
	return (deg * PI / 180.0);
}

float deg(float rad){
	return (rad * 180.0 / PI);
}
