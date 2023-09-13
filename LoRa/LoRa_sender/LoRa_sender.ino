#include <Wire.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"
// LAPTOP
#include "C:\Users\jocke\OneDrive\Skrivbord\GitHub\LoRa-communication\RAK811.h"
#include "C:\Users\jocke\OneDrive\Skrivbord\GitHub\LoRa-communication\RAK811.cpp"

// STATIONARY
//#include "C:\Users\jocke\Desktop\GitStuff\LoRa-communication\RAK811.h"
//#include "C:\Users\jocke\Desktop\GitStuff\LoRa-communication\RAK811.cpp"

// For debugging
#define DEBUG 1

#if DEBUG == 1
#define debugln(x) Serial.println(x)
#define debug(x) Serial.print(x)
#else
#define debugln(x)
#define debug(x)
#endif

#define TXpin 11 
#define RXpin 10
#define WORK_MODE LoRaP2P   //  LoRaWAN or LoRaP2P
#define TXpin 11   // Set the virtual serial port pins
#define RXpin 10
SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port
int RESET_PIN = 12;
int ERROR_PIN = 13;
int SERIAL_AVAILABLE_PIN = 9;
RAK811 RAKLoRa(RAKSerial,Serial);

#define can_tx  8           // tx of serial can module connect to D2
#define can_rx  9           // rx of serial can module connect to D3

SoftwareSerial can_serial(can_tx, can_rx);

#define uart_can can_serial

// CAN Variables
unsigned long __id = 0;
unsigned char __ext = 0; // extended frame or standard frame
unsigned char __rtr = 0; // remote frame or data frame
unsigned char __fdf = 0; // can fd or can 2.0
unsigned char __len = 0; // data length
unsigned char __dta[8]; // data

unsigned long lastTimeLORASent = millis();

// Motor Controller
double HeatsinkTemp = 0.0;
double MotorTemp = 0.0;

double BusCurrent = 0.0;
double BusVoltage = 0.0;

double MotorVelocity = 0.0;
double VehicleVelocity = 0.0;

// BMS (Orion 2)

double PackVoltage = 0.0;
double PackCurrent = 0.0;
double PackAverageCurrent = 0.0;

double PackSOH = 0.0;
double PackSOC = 0.0;

double LowCellVoltage = 0.0;
double HighCellVoltage = 0.0;
double AvgCellVoltage = 0.0;

double HighTemperature = 0.0;
double LowTemperature = 0.0;
double AvgTemperature = 0.0;
double InternalTemperature = 0.0;

// MPPT (SEC-B175-7A TPEE)
double MPPTInputVoltage = 0.0;
double MPPTInputCurrent = 0.0;

double MPPTOutputVoltage = 0.0;
double MPPTOutputCurrent = 0.0;

// Insulator (ISO165C-1 Bender)


void uart_init(unsigned long baudrate)
{
    uart_can.begin(baudrate);
}

void uart_write(unsigned char c)
{
    uart_can.write(c);
}

unsigned char uart_read()
{
    return uart_can.read();
}

int uart_available()
{
    return uart_can.available();
}

void set_uart_LoRa(int current_LoRa_baud, int new_LoRa_baud)
{
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  delay(400);
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
  Serial.println("Reset RAK");
  delay(400);

  RAKSerial.begin(current_LoRa_baud); // Arduino Shield
  String uart_status = RAKLoRa.rk_setUARTConfig(new_LoRa_baud, 8, 0, 0, 0);
  Serial.println("UART SET STATUS: " + uart_status);
  
  delay(400);
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  delay(400);
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
}


// For motor controller
// Function to extract a specified number of bytes from a string (IEEE754) and convert to decimal number
double extractBytesToDecimal(String data, int startByte, int numBytes) {

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
  double MPPTdata = 0;
  for(int i = startIndex; i < data.length(); i++)
  {

    String data_substr = data.substring(i, i+1);


    if(data_substr == " ")
    {
      byteCounter++;

      if(byteCounter == 1)
      {
        MPPTdata = byte_data.toDouble();
        break;
      }
    }
    else
    {
      byte_data += data_substr; 
    }
  }

  return MPPTdata;
}

double extractDataMPPT(String data, int startByte, int numBytes)
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
  //debugln("Start index: " + String(startIndex));

  byteCounter = 0;
  String byte_data = "";
  double MPPTdata = 0;
  for(int i = startIndex; i < data.length(); i++)
  {

    String data_substr = data.substring(i, i+1);

    if(byteCounter == numBytes)
    {
      break;
    }
    else if(data_substr == " ")
    {
      byteCounter++;

      if(byteCounter == 1)
      {
        MPPTdata += 256*byte_data.toDouble();
      }
      else
      {
        MPPTdata += byte_data.toDouble();
      }
      byte_data = "";
    }
    else
    {
      byte_data += data_substr; 
    }
  }

  return MPPTdata;
}

void update_data(String ID, String data)
{

  debugln("ID: " + ID + ", Data: " + data);

  if(ID == "1025") //0x401 Status Information
  {
  }

  if(ID == "1026") //0x402 Bus Measurement
  {

  }

  if(ID == "1027") //0x403 Velocity Measurement
  {
    double motor_velocity = extractBytesToDecimal(data, 0, 4);
    double vehicle_velocity = extractBytesToDecimal(data, 4, 4);
    debugln();

    debugln("Vehicle vel: " + String(vehicle_velocity));
    debugln("Motor vel: " + String(motor_velocity));
    
 
  }

  if(ID == "1035") //0x40B MC Temperatures
  {
    double heat_sink_temp = extractBytesToDecimal(data, 0, 4);
    double motor_temp = extractBytesToDecimal(data, 4, 4);

    debugln("Heat-sink temp: " + String(heat_sink_temp));
    debugln("Motor temp: " + String(motor_temp));
  }

  // ---------------- BMS ------------------------
  if(ID == "1536")
  {
    PackCurrent = 0.1*extractDataMPPT(data, 0, 2);
    PackVoltage = 0.1*extractDataMPPT(data, 2, 2);
    PackAverageCurrent = 0.1*extractDataMPPT(data, 4, 2);
    PackSOH = extractSingleByte(data, 6);
    PackSOC = extractSingleByte(data, 7);

    debugln("PackCurrent: " + String(PackCurrent));
    debugln("PackVoltage: " + String(PackVoltage));
    debugln("PackAverageCurrent: " + String(PackAverageCurrent));
    debugln("PackSOH: " + String(PackSOH));
    debugln("PackSOC: " + String(PackSOC));
  }

  if(ID == "1537") //BMS VOLTAGES
  {
    LowCellVoltage = 0.0001*extractDataMPPT(data, 0, 2);
    HighCellVoltage = 0.0001*extractDataMPPT(data, 2, 2);
    AvgCellVoltage = 0.0001*extractDataMPPT(data, 4, 2);

    debugln("LowCellVoltage: " + String(LowCellVoltage));
    debugln("HighCellVoltage: " + String(HighCellVoltage));
    debugln("AvgCellVoltage: " + String(AvgCellVoltage));
   
  }

  if(ID == "1538") //BMS TEMPERATURES
  {
    HighTemperature = extractSingleByte(data, 0);
    LowTemperature = extractSingleByte(data, 1);
    AvgTemperature = extractSingleByte(data, 2);
    InternalTemperature = extractSingleByte(data, 3);

    debugln("HighTemperature: " + String(HighTemperature));
    debugln("LowTemperature: " + String(LowTemperature));
    debugln("AvgTemperature: " + String(AvgTemperature));
    debugln("InternalTemperature: " + String(InternalTemperature));
  }
  
  // ----------------- MPPT ----------------------
  if(ID == "512") 
  { // InputVolt, InputCurrent, OutputVolt, OutputCurrent
    MPPTInputVoltage = 0.01*extractDataMPPT(data, 0, 2);
    MPPTInputCurrent = 0.0005*extractDataMPPT(data, 2, 2);
    MPPTOutputVoltage = 0.01*extractDataMPPT(data, 4, 2);
    MPPTOutputCurrent = 0.0005*extractDataMPPT(data, 6, 2);

    /*
    debugln("MPPT IN_VOLT: " + String(MPPTInputVoltage));
    debugln("MPPT IN_CURRENT: " + String(MPPTInputVoltage));
    debugln("MPPT OUT_VOLT: " + String(MPPTOutputVoltage));
    debugln("MPPT OUT_CURRENT: " + String(MPPTOutputCurrent));
    */
  }

  if(ID == "513") //
  {// Errors

  }


  // ----- HÄR ÄR DET BUGGIGT, LORAn KRASCHAR NÄR DENNA KODEN KÖRS 
  // ----- TROLIGEN P.GA noInterrupts() INTE FAKTISKT STOPPAR I2C INTERRUPT
  // ----- FRÅN ECUn OCH NÄR DEN FÖRSÖKER SKICKA KOMMER EN INTERRUPT  
  //noInterrupts();
  //char loradata[] = "1337";
  //RAKLoRa.rk_sendP2PData(1, "10", loradata);
  //interrupts();

  /* -------------------------------------------------------------- */
}

String convertCANMsgToLoRa(const int can_id, const unsigned char* can_data, const unsigned int can_data_size) {
  String newLoraMsg = String(can_id);
  for (int i = 0; i < can_data_size; i++) {
    newLoraMsg += String(can_data[i]);
  }

  return newLoraMsg;
}


void setup() 
{
  pinMode(RESET_PIN, OUTPUT);
  pinMode(ERROR_PIN, OUTPUT);
  pinMode(SERIAL_AVAILABLE_PIN, OUTPUT);
  pinMode(9, INPUT);
  
  Serial.begin(9600);
  
  while(Serial.read()>= 0) {}  
  while(!Serial);

  Serial.println("StartUP");

  int current_baud = 9600;
  set_uart_LoRa(current_baud, 9600); //Sets UART -> "at+uart=31250,8,0,0,0"

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

  //String setUART = RAKLoRa.rk_setUARTConfig(9600, 8, 0, 0, 0);
  //DebugSerial.println("UART conf.   successful: " + String(setUART));
  Serial.setTimeout(5);
  RAKSerial.setTimeout(5);

  /* SETUP OF CAN MODULE */
  uart_init(9600);
  can_speed_20(500000);          // set can bus baudrate to 500k
}


void loop() 
{

  if(read_can(&__id, &__ext, &__rtr, &__fdf, &__len, __dta))
    {
        Serial.print("GET DATA FROM: 0x");
        Serial.println(__id, HEX);
        Serial.print("EXT = ");
        Serial.println(__ext);
        Serial.print("RTR = ");
        Serial.println(__rtr);
        Serial.print("FDF = ");
        Serial.println(__fdf);
        Serial.print("LEN = ");
        Serial.println(__len);
        
        for(int i=0; i<__len; i++)
        {
            Serial.print(__dta[i]);
            Serial.print(' ');
        }
        Serial.println();t

        if (millis() - lastTimeLORASent >= 500) {
          /*String msgToSend = convertCANMsgToLoRa(__id, __dta, __len);
          debugln("Lora msg to send: " + msgToSend);
          
          char data[__len];
          // make sure that the new string is null terminated
          //data[__len] = '\0';

          for (int i = 0; i < __len; i++) 
          {
              data[i] = __dta[i];
          }
          RAKLoRa.rk_sendP2PData(2, "100", data);
          */
          RAKLoRa.rk_sendP2PData(1, "10", "1337");
          lastTimeLORASent = millis();
        }
    }

  //Serial.println("Doin loop");
/*
  if(Serial.available())
  {
    String input = Serial.readStringUntil(".");  // Read the data from the serial port
    Serial.flush();
    char data[input.length()+1];
    Serial.println(input);
   
      // make sure that the new string is null terminated
    data[input.length()] = '\0';
     
    for (int i = 0; i < input.length(); i++) 
    {
        data[i] = input[i];
    }

    // data must > 10
    RAKLoRa.rk_sendP2PData(1, "10", data);
    digitalWrite(ERROR_PIN, HIGH);
    Serial.println(data);
  }
  else
  {
    char data[] = "305069";
    RAKLoRa.rk_sendP2PData(1, "10", data);
    delay(200);
  }
  */
}


