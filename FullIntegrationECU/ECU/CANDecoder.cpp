#include <Arduino.h>
#include <string.h>
#include "CANDecoder.h"
#include "GlobalVariablesReceive.h"


// Necessary variable for cruise control
unsigned long lastTimePointVelocityFetched = 0;

// CANBUS Data
double heatsinkTemp = 0.0;
double motorTemp = 0.0;
double busCurrent = 0.0;
double busVoltage = 0.0;
double motorVelocity = 0.0;
double vehicleVelocity = 0.0;
double packVoltageMax = 159.6;
double packVoltage = 0.0;
double packCurrent = 0.0;
double packAverageCurrent = 0.0;
double lowCellVoltage = 0.0;
double highCellVoltage = 0.0;
double avgCellVoltage = 0.0;
double highTemperature = 0.0;
double lowTemperature = 0.0;
double avgTemperature = 0.0;
double internalTemperature = 0.0;
double MPPTInputVoltage = 0.0;
double MPPTInputCurrent = 0.0;
double MPPTOutputVoltage = 0.0;
double MPPTOutputCurrent = 0.0;
double MMPTOutputPower = 0.0;
static double MPPT_power_max = 2000;



void DecodeCANMsg(long __id, const char* __dta) {
    
    /*********** Motor Controller ***********/
    if (__id == 1025) {} // Status information
    else if (__id == 1026) {
        double busCurrent = extractBytesToDecimal(__dta, 0, 4);
        double busVoltage = extractBytesToDecimal(__dta, 4, 4);
        lastTimePointVelocityFetched = millis();
    } // Bus measurement
    else if (__id == 1027) { // Velocity Measurement
        double motorVelocity = extractBytesToDecimal(__dta, 0, 4);
        double vehicleVelocity = extractBytesToDecimal(__dta, 4, 4);
        lastTimePointVelocityFetched = millis();
    }
    else if (__id == 1035) {
        double heatsinkTemp = extractBytesToDecimal(__dta, 0, 4);
        double motorTemp = extractBytesToDecimal(__dta, 4, 4);
    }
    /*********** Battery Management System ***********/
    else if (__id == 1536) {
        packCurrent = 0.1*extractDataNrBytes(__dta, 0, 2);
        packVoltage = 0.1*extractDataNrBytes(__dta, 2, 2);
        packAverageCurrent = 0.1*extractDataNrBytes(__dta, 4, 2);
        //PackSOH = extractSingleByte(data, 6);
        //PackSOC = extractSingleByte(data, 7);
    }
    else if (__id == 1537) {
        lowCellVoltage = 0.0001*extractDataNrBytes(__dta, 0, 2);
        highCellVoltage = 0.0001*extractDataNrBytes(__dta, 2, 2);
        avgCellVoltage = 0.0001*extractDataNrBytes(__dta, 4, 2);
    }
    else if (__id == 1538) {

    }
    /*********** MPPT***********/
    else if (__id == 513) {
        MPPTInputVoltage = 0.01*extractDataNrBytes(__dta, 0, 2);
        MPPTInputCurrent = 0.0005*extractDataNrBytes(__dta, 2, 2);
        MPPTOutputVoltage = 0.01*extractDataNrBytes(__dta, 4, 2);
        MPPTOutputCurrent = 0.0005*extractDataNrBytes(__dta, 6, 2);
        MMPTOutputPower = MPPTOutputVoltage*MPPTOutputCurrent;
    }

}

// For motor controller
// Function to extract a specified number of bytes from a string (IEEE754) and convert to decimal number
double extractBytesToDecimal(const char* __dta, int startByte, int numBytes) {
  String data = String(__dta);
  // Calculate startbyte index position ex. startByte: 4 = index: 14 (65 160 0 0 68 (250 0 0 1027))
  int startIndex = 0;
  int byteCounter = 0; // Bytes inc. for each " "
  for(int i = 0; i < data.length(); i++)
  {

    if(byteCounter == startByte)
    {
      startIndex = i;
      break;
    }

    if(data.substring(i, i+1) == " ")
    {
      byteCounter++;
    }
  
  }

  //debugln("Start index: " + String(startIndex));

  byte bytes[numBytes];

  byteCounter = 0;
  String byte_data = "";
  for(int i = startIndex; i < data.length(); i++)
  {

    String data_substr = data.substring(i, i+1);

    if(byteCounter == numBytes)
    {
      break;
    }
    else if(data_substr == " ")
    {
      //debugln(byte_data);
      bytes[byteCounter] = (byte) strtoul(byte_data.c_str(), NULL, 10);
      byteCounter++;
      byte_data = "";
    }
    else
    {
      byte_data += data_substr; 
    }

  }
  /* For debugging of output bytes
  for(int i = 0; i < numBytes; i++)
  {
    debug(bytes[i]);
    debug(" ");
  }
  debugln();
  */
  
  double value;
  memcpy(&value, bytes, numBytes);
  // Return the decimal value
  return value;
}


double extractDataNrBytes(const char* __dta, int startByte, int numBytes)
{
  String data = String(__dta);
  // Calculate startbyte index position ex. startByte: 4 = index: 14 (65 160 0 0 68 (250 0 0 1027))
  int startIndex = 0;
  int byteCounter = 0; // Bytes inc. for each " "
  for(int i = 0; i < data.length(); i++)
  {

    if(byteCounter == startByte)
    {
      startIndex = i;
      break;
    }

    if(data.substring(i, i+1) == " ")
    {
      byteCounter++;
    }
  
  }
  //debugln("Start index: " + String(startIndex));

  byteCounter = 0;
  String byteData = "";
  double finalByteData = 0;
  for(int i = startIndex; i < data.length(); i++)
  {

    String dataSubstr = data.substring(i, i+1);

    if(byteCounter == numBytes)
    {
      break;
    }
    else if(dataSubstr == " ")
    {
      byteCounter++;

      if(byteCounter == 1)
      {
        finalByteData += 256*byteData.toDouble();
      }
      else
      {
        finalByteData += byteData.toDouble();
      }
      byteData = "";
    }
    else
    {
      byteData += dataSubstr; 
    }
  }

  return finalByteData;
}