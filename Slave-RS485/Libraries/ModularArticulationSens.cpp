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
#define motorCurrentSensor A0
#define supplyVoltageSensor A1	// ONLY POSITIVE VOLTAGES
#define motorVoltageSensor A2	// POSITIVE AND NEGATIVE VOLTAGES

// DIGITAL INPUTS
#define motorTempSensor 7 // D7

// TEMPERATURE - INITIALIZATION
OneWire oneWire(motorTempSensor);	// One Wire Comunication
DallasTemperature tempSensor(&oneWire);

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



