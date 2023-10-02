#include "RAK811.h"
#include <SoftwareSerial.h>



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


  //String setUART = RAKLoRa.rk_setUARTConfig(9600, 8, 0, 0, 0);
  //DebugSerial.println("UART conf. successful: " + String(setUART));
}

void loop() 
{
  int available = RAKSerial.available();
  if (available) {
  Serial.println("Data available: " + String(available));
  String data = RAKSerial.readStringUntil("\n");
  Serial.println("LoRa data: " + data);
  }

  

/*
  if(RAKSerial.available())
  {
    String filtered_data = "";
    String data = RAKSerial.readStringUntil("\n");

    // Filters data "at+recv=0,0,-73,24,2,1029" -> filtered_data "1029"
    for(int i = data.length()-1; i >= 0; i--)
    {
      String substring = data.substring(i-1,i);

      if(substring.equals(","))
      {
        break;
      }
      else
      {
        filtered_data = substring + filtered_data;
      }
    }
    filtered_data = remove_chars(filtered_data);
    Serial.println(filtered_data);
        
  }
*/
}

