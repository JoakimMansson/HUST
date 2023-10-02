#pragma once

/*
extern int gasPotential;
extern int brakePotential;
extern int drivingMode;
extern int inCruising;
extern int incDecCruiseSpeed;
extern int activateHorn;
extern int activateRightBlinker;
extern int activateLeftBlinker;
*/


// Necessary variable for cruise control
extern unsigned long lastTimePointVelocityFetched; 
extern unsigned long lastTimePointBusCurrentFetched; 

// CANBUS Data
extern double heatsinkTemp;
extern double motorTemp;
extern double busCurrent;
extern double busVoltage;
extern double motorVelocity;
extern double vehicleVelocity;
extern double packVoltageMax;
extern double packVoltage;
extern double packCurrent;
extern double packAverageCurrent;
extern double lowCellVoltage;
extern double highCellVoltage;
extern double avgCellVoltage;
extern double highTemperature;
extern double lowTemperature;
extern double avgTemperature;
extern double internalTemperature;
extern double MPPTInputVoltage;
extern double MPPTInputCurrent;
extern double MPPTOutputVoltage;
extern double MPPTOutputCurrent;
extern double MMPTOutputPower;
extern double MPPT_power_max;