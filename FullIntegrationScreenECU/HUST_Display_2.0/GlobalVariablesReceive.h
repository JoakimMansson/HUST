#pragma once

// Necessary variable for cruise control
extern unsigned long lastTimePointVelocityFetched; 
extern unsigned long lastTimePointBusCurrentFetched; 

// CANBUS Data
extern int heatsinkTemp;
extern int motorTemp;
extern double busCurrent;
extern double busVoltage;
extern double motorVelocity;
extern int vehicleVelocity;
extern double packVoltageMax;
extern double packVoltage;
extern double packCurrent;
extern double packAverageCurrent;
extern double lowCellVoltage;
extern double highCellVoltage;
extern double avgCellVoltage;
extern int highTemperature;
extern int lowTemperature;
extern int avgTemperature;
extern int internalTemperature;
extern double MPPTInputVoltage;
extern double MPPTInputCurrent;
extern double MPPTOutputVoltage;
extern double MPPTOutputCurrent;
extern double MMPTOutputPower;
extern double MPPT_power_max;