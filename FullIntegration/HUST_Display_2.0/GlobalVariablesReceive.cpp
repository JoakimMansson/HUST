#include "GlobalVariablesReceive.h"

#pragma once

// CANBUS Data
double HeatsinkTemp = 0.0;
double MotorTemp = 0.0;
double BusCurrent = 0.0;
double BusVoltage = 0.0;
double MotorVelocity = 0.0;
double VehicleVelocity = 0.0;
double PackVoltageMax = 159.6;
double PackVoltage = 0.0;
double PackCurrent = 0.0;
double PackAverageCurrent = 0.0;
double LowCellVoltage = 0.0;
double HighCellVoltage = 0.0;
double AvgCellVoltage = 0.0;
double HighTemperature = 0.0;
double LowTemperature = 0.0;
double AvgTemperature = 0.0;
double InternalTemperature = 0.0;
double MPPTInputVoltage = 0.0;
double MPPTInputCurrent = 0.0;
double MPPTOutputVoltage = 0.0;
double MPPTOutputCurrent = 0.0;
double MMPTOutputPower = 0.0;
static double MPPT_power_max = 2000;