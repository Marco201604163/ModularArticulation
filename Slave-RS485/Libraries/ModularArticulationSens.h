/*
Created on Sat Mar  13 11:23:10 2021

@author: MarcoRocha
*/

#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

void startArticulationSens();

// RETURNS SENSORS VALUES
float getSupplyVoltage();	// (V)
float getMotorVoltage();	// (V)
float getMotorTemp();		// (Celsius)