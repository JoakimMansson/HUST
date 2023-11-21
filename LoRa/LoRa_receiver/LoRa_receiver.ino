#include <SoftwareSerial.h>
#include "RAK811.h"


#define TXpin 11
#define RXpin 10
#define WORK_MODE LoRaP2P   //  LoRaWAN or LoRaP2P

SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port
int RESET_PIN = 12;
int RECEIVED_PIN = 13;
RAK811 RAKLoRa(RAKSerial,Serial);

unsigned long lastTimeCANSent = millis();

void setUART(int current_LoRa_baud, int new_LoRa_baud)
{
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  delay(400);
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
  Serial.println("Reset RAK");
  delay(400);

  //Start communication
  RAKSerial.begin(current_LoRa_baud); // Arduino Shield
  String uart_status = RAKLoRa.rk_setUARTConfig(new_LoRa_baud, 8, 0, 0, 0);
  Serial.println("UART SET STATUS: " + uart_status);
  
  delay(400);
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  delay(400);
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
}


String remove_chars(String input_str)
{
  String new_str = "";
  for(int i = 0; i < input_str.length(); i++)
  {
    unsigned int char_ascii_value = input_str.charAt(i);
    if(char_ascii_value >= 48 && char_ascii_value <=  57)
    {
      new_str += input_str.substring(i, i+1);
    }
  }

  return new_str;
}

String extractDataAfter5thComma(String input) {
  int commaCount = 0;
  int startIndex = 0;

  // Iterate through each character in the input string
  for (int i = 0; i < input.length(); i++) {
    if (input.charAt(i) == ',') {
      commaCount++;

      // If we have found the 5th comma, store its index
      if (commaCount == 5) {
        startIndex = i + 1;
        break;
      }
    }
  }

  // Extract the substring after the 5th comma
  String extractedData = input.substring(startIndex);

  return extractedData;
}


int hexStringToInt(const String &hexValue) {
  if (hexValue.length() != 6) {
    // Ensure the input string has the correct length (6 characters).
    return -1; // Return an error code (you can choose a different error code if needed).
  }

  // Extract the different parts of the hex string.
  String idHex = hexValue.substring(0, 2);
  String intHex = hexValue.substring(2, 5);
  String decimalHex = hexValue.substring(5);

  // Convert each part to integers.
  int id = strtol(idHex.c_str(), NULL, 16);
  int intValue = strtol(intHex.c_str(), NULL, 16);
  int decimalValue = strtol(decimalHex.c_str(), NULL, 16);

  // Combine the parts to form the final integer value.
  int result = (id << 16) | (intValue << 8) | decimalValue;

  return result;
}


void setup() 
{
  pinMode(RESET_PIN, OUTPUT);
  pinMode(RECEIVED_PIN, OUTPUT);
  
  Serial.begin(9600);
  
  while(Serial.read()>= 0) {}  
  while(!Serial);

  Serial.println("StartUP");

  int current_baud = 9600;
  setUART(current_baud, 9600); //Sets UART -> "at+uart=31250,8,0,0,0"

  RAKSerial.begin(9600); // Arduino Shield
  delay(200);
  Serial.println(RAKLoRa.rk_getBand());
  delay(200);

  Serial.println("Current version: " + RAKLoRa.rk_currentVersion());

  /*   Workmode 0 = LoRaWAN, 1 = P2P    */
  Serial.println("Initializing workmode");
 
  RAKLoRa.rk_setWorkingMode(1); //Sets work mode to P2P
  Serial.println("Current mode: " + RAKLoRa.rk_getCurrentMode());
  delay(200);
  Serial.println("Work mode initialized");


  Serial.println("Initializing p2p");
  String P2pInitialized = RAKLoRa.rk_initP2P("869525000", "9", "2", "1", "8", "20");
  Serial.println("P2P initialized: " + P2pInitialized);

  
  Serial.println("Receiving packets status: " + RAKLoRa.rk_recvP2PData(1));

  RAKSerial.setTimeout(5);
  Serial.setTimeout(5);
  
  Serial.println("START");
  //String setUART = RAKLoRa.rk_setUARTConfig(9600, 8, 0, 0, 0);
  //DebugSerial.println("UART conf. successful: " + String(setUART));
}

void loop() 
{
  int available = RAKSerial.available();
  if (available)
  {
    String data = RAKSerial.readStringUntil("\n");
    
    data = data.substring(0, data.length()-2);
    String hexValue = extractDataAfter5thComma(data);
    //Serial.println(hexValue);

    
    // Extract the different parts of the hex string.
    String idHex = hexValue.substring(0, 2);
    String intHex = hexValue.substring(2, 5);
    String decimalHex = hexValue.substring(5);
  

    // Convert each part to integers.
    long id = strtol(idHex.c_str(), NULL, 16);
    long integer = strtol(intHex.c_str(), NULL, 16);
    long decimal = strtol(decimalHex.c_str(), NULL, 16);

  
    if (id == 2) {
      Serial.println("Heatsink_temp: " + String(integer) + "." + String(decimal));
    }
    else if (id == 3) {
      Serial.println("Motor_temp: " + String(integer) + "." + String(decimal));
    }
    else if (id == 4) {
      Serial.println("Internal_temp: " + String(integer) + "." + String(decimal));
    }
    else if (id == 5) {
      Serial.println("High_temp: " + String(integer) + "." + String(decimal));
    }
    else if (id == 6) {
      Serial.println("Pack_Current: " + String(integer) + "." + String(decimal));
    }
    else if (id == 7) {
      Serial.println("Pack_open_voltage: " + String(integer) + "." + String(decimal));
    }
    else if (id == 8) {
      Serial.println("Low_cell_voltage: " + String(integer) + "." + String(decimal));
    }
    else if (id == 9) {
      Serial.println("Bus_Current: " + String(integer) + "." + String(decimal));
    }
    
  }
}

