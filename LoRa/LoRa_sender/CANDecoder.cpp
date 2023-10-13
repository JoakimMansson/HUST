#include <Arduino.h>
#include <string.h>
#include "CANDecoder.h"

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


double extractSingleByte(String data, int startByte)
{
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

  byteCounter = 0;
  String byte_data = "";
  double finalByteData = 0;
  for(int i = startIndex; i < data.length(); i++)
  {

    String data_substr = data.substring(i, i+1);


    if(data_substr == " ")
    {
      byteCounter++;

      if(byteCounter == 1)
      {
        finalByteData = byte_data.toDouble();
        break;
      }
    }
    else
    {
      byte_data += data_substr; 
    }
  }

  return finalByteData;
}