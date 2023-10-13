#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"
#include "CANDecoder.h"
#include "RAK811.h"


// For debugging
#define DEBUG 1

#if DEBUG == 1
#define debugln(x) Serial.println(x)
#define debug(x) Serial.print(x)
#else
#define debugln(x)
#define debug(x)
#endif

#define WORK_MODE LoRaP2P   //  LoRaWAN or LoRaP2P
#define TXpin 11   // Set the virtual serial port pins
#define RXpin 10
SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port
int RESET_PIN = 12;
int ERROR_PIN = 13;
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

String intToStringHex(const int integer) {
  return String(integer, HEX);
}

void sendLoraMsg(const int __id, const double __dta) {
  String id = String(__id, HEX);
  int integerPart = int(__dta);
  int decimalPart = int((__dta - integerPart) * 10); // Assuming two decimal places
  
  String integerHex = String(integerPart, HEX);
  String decimalHex = String(decimalPart, HEX);
  while (id.length() < 2) {
    id = "0" + id;
  }

  // Pad the integer and decimal parts to ensure they are three and two characters long respectively
  while (integerHex.length() < 3) {
    integerHex = "0" + integerHex; // PAD INSTEAD WITH g AT INTEGER, LORA CANT SEND 0
  }
  while (decimalHex.length() < 1) {
    decimalHex = "0 " + decimalHex; // PAD INSTEAD WITH n AT INTEGER, LORA CANT SEND 0
  }
  /*
  * Max id representation: 255 (ff)
  * Max integer representation: 4095 (fff)
  * Max decimal representation: 15 (f)
  */
  
  String loraData = id + integerHex + decimalHex;
  //Serial.println("Int hex: " + integerHex);
  //Serial.println("Int decimal: " + decimalHex);
  //Serial.println("Lora data: " + loraData);
  
  // Combine the integer and decimal parts with a period (.) in between
  RAKLoRa.rk_sendP2PData(1, "10", loraData.c_str());
  delay(300);
}


void setup() 
{
  pinMode(RESET_PIN, OUTPUT);
  pinMode(ERROR_PIN, OUTPUT);
  
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
  static unsigned long lastTimeBusCurrent = millis();
  static unsigned long lastTimeBusVoltage = millis();

  static unsigned long lastTimeMotorVelocity = millis();
  static unsigned long lastTimeVehicleVelocity = millis();

  static unsigned long lastTimeHeatsinkTemp = millis();
  static unsigned long lastTimeMotorTemp = millis();

  static unsigned long lastTimePackCurrent = millis();
  static unsigned long lastTimePackVoltage = millis();
  static unsigned long lastTimePackAverageCurrent = millis();
  static unsigned long lastTimePackSOH = millis();
  static unsigned long lastTimePackSOC = millis();

  static unsigned long lastTimeLowCellVoltage = millis();
  static unsigned long lastTimeHighCellVoltage = millis();
  static unsigned long lastTimeAvgCellVoltage = millis();

  static unsigned long lastTimeHighTemperature = millis();
  static unsigned long lastTimeLowTemperature = millis();
  static unsigned long lastTimeAvgTemperature = millis();
  static unsigned long lastTimeInternalTemperature = millis();

  static unsigned long lastTimeMPPTInputVoltage = millis();
  static unsigned long lastTimeMPPTInputCurrent = millis();
  static unsigned long lastTimeMPPTOutputVoltage = millis();
  static unsigned long lastTimeMPPTOutputCurrent = millis();
  static unsigned long lastTimeMPPTOutputPower = millis();

  /* BMS CAN MESSAGES
  * 0x600 [PACK_CURRENT, PACK_OPEN_VOLTAGE, CURRENT_LIMIT_STATUS, AVG_CURRENT, LOW_CELL_VOLTAGE, HIGH_CELL_VOLTAGE, INPUT_SUPPLY_VOLTAGE, RELAY STATE]
  * 0x601 [INTAKE_TEMP, INTERNAL_TEMP, HIGH_TEMP, LOW_TEMP, AVG_TEMP, PACK_AMP_HOURS, HIGH_THERMISTOR_ID, LOW_THERMISTOR_ID]
  */
  /* ALL THE ENCODINGS FOR 
    encodings = {
            #BMS_Pack
            10: "PackCurrent",
            11: "PackVoltage",
            12: "PackStateOfHealth",
            13: "AvgPackCurrent",
            14: "PackStateOfCharge",
            #BMS_Cell
            15: "LowCellVoltage",
            16: "HighCellVoltage",
            17: "AvgCellVoltage",
            #BMS_Failsafes
            18: "VoltageFailsafeActive",
            19: "CurrentFailsafeActive",
            20: "RelayFailsafeActive",
            21: "CellBalancingActive",
            22: "ChangeinterlockFailsafeActive",
            23: "ThermistorB_valueTableInvalid",
            24: "InputPowerSupplyFailed",
            #BMS_Temperatures
            25: "HighestTemperature",
            26: "LowestTemperature",
            27: "AverageTemperature",
            28: "InternalTemperature",

            #MC_Temperatures
            29: "HeatsinkTemperature",
            30: "MotorTemperature",
            #MC_CurrentVoltage
            31: "BusCurrent",
            32: "BusVoltage",
            #MC_Velocity
            33: "MotorVelocity",
            34: "VehicleVelocity",
            #MC_ErrorFlags
            35: "MotorOverSpeed",
            36: "DesaturationFault",
            37: "RailUnderVoltage",
            38: "ConfigReadError",
            39: "WatchdogCausedLastReset",
            40: "BadMotorPositionHallSequence",
            41: "DCBusOverVoltage",
            42: "SoftwareOverCurrent",
            43: "HardwareOverCurrent",
            #MC_LimitFlags
            44: "IPMTemperatureOrMotorTemperature",
            45: "BusVoltageLowerLimit",
            46: "BusVoltageUpperLimit",
            47: "BusCurrentLimit",
            48: "Velocity",
            49: "MotorCurrent",
            50: "OutputVoltagePWM"
        }
  
  */

  /*
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
        


        Serial.println(atoi(__dta));

        // ------ Motor Controller ------ //
      if (__id == 1025) {} // Status information
      else if (__id == 1026) {
        double busCurrent = extractBytesToDecimal(__dta, 0, 4);
        double busVoltage = extractBytesToDecimal(__dta, 4, 4);

        if (millis() - lastTimeBusCurrent > 10000) {
          sendLoraMsg(31, busCurrent);
          lastTimeBusCurrent = millis();
        }
        if (millis() - lastTimeBusVoltage > 10000) {
          sendLoraMsg(32, busVoltage);
          lastTimeBusVoltage = millis();
        }
      } // Bus measurement
      else if (__id == 1027) { // Velocity Measurement
        double motorVelocity = extractBytesToDecimal(__dta, 0, 4);
        double vehicleVelocity = extractBytesToDecimal(__dta, 4, 4);

        if (millis() - lastTimeMotorVelocity > 60000) { // motorVelocity @60 SECONDS
          sendLoraMsg(33, motorVelocity);
          lastTimeVehicleVelocity = millis();
        }
        if (millis() - lastTimeVehicleVelocity > 30000) { // vehicleVelocity @30 SECONDS
          sendLoraMsg(34, vehicleVelocity);
          lastTimeVehicleVelocity = millis();
        }
      }
      else if (__id == 1035) {
        double heatsinkTemp = extractBytesToDecimal(__dta, 0, 4);
        double motorTemp = extractBytesToDecimal(__dta, 4, 4);

        if (millis() - lastTimeHeatsinkTemp > 20000) { // heatsinkTemp @20 SECONDS
          sendLoraMsg(29, heatsinkTemp);
          lastTimeHeatsinkTemp = millis();
        }
        if (millis() - lastTimeMotorTemp > 10000) { // motorTemp @10 SECONDS
          sendLoraMsg(30, motorTemp);
          lastTimeMotorTemp = millis();
        }
      }
      // ------------ Battery Management System ------------ //
      else if (__id == 1536) {
        double packCurrent = 0.1*extractDataNrBytes(__dta, 0, 2);
        double packVoltage = 0.1*extractDataNrBytes(__dta, 2, 2);
        double packAverageCurrent = 0.1*extractDataNrBytes(__dta, 4, 2);
        double packSOH = extractSingleByte(__dta, 6);
        double packSOC = extractSingleByte(__dta, 7);
        

        if (millis() - lastTimePackCurrent > 20000) { // packCurrent @20 SECONDS
          sendLoraMsg(10, packCurrent);
          lastTimePackCurrent = millis();
        }
        if (millis() - lastTimePackVoltage > 6000) { // packVoltage @6 SECONDS
          sendLoraMsg(11, packVoltage);
          lastTimePackVoltage = millis();
        }
        if (millis() - lastTimePackAverageCurrent > 30000) { // packAverageCurrent @30 SECONDS
          sendLoraMsg(13, packAverageCurrent);
          lastTimePackAverageCurrent = millis();
        }
        if (millis() - lastTimePackSOH > 120000) { // packSOH @120 SECONDS
          sendLoraMsg(12, packSOH);
          lastTimePackSOH = millis();
        }

      }
      else if (__id == 1537) {
        double lowCellVoltage = 0.0001*extractDataNrBytes(__dta, 0, 2);
        double highCellVoltage = 0.0001*extractDataNrBytes(__dta, 2, 2);
        double avgCellVoltage = 0.0001*extractDataNrBytes(__dta, 4, 2);

        if (millis() - lastTimeLowCellVoltage > 8000) { // lowCellVoltage @8 SECONDS
          sendLoraMsg(15, lowCellVoltage);
          lastTimeLowCellVoltage = millis();
        }
        if (millis() - lastTimeHighCellVoltage > 8000) { // highCellVoltage @8 SECONDS
          sendLoraMsg(16, highCellVoltage);
          lastTimeHighCellVoltage = millis();
        }
        if (millis() - lastTimeAvgCellVoltage > 5000) { // avgCellVoltage @5 SECONDS
          sendLoraMsg(17, avgCellVoltage);
          lastTimeAvgCellVoltage = millis();
        }
      }
      else if (__id == 1538) {
        double highTemperature = extractSingleByte(__dta, 0);
        double lowTemperature = extractSingleByte(__dta, 1);
        double avgTemperature = extractSingleByte(__dta, 2);
        double internalTemperature = extractSingleByte(__dta, 3);

        if (millis() - lastTimeHighTemperature > 8000) { // highTemperature @8 SECONDS
          sendLoraMsg(25, highTemperature);
          lastTimeHighTemperature = millis();
        }
        if (millis() - lastTimeLowTemperature > 8000) { // lowTemperature @8 SECONDS
          sendLoraMsg(26, lowTemperature);
          lastTimeLowTemperature = millis();
        }
        if (millis() - lastTimeAvgTemperature > 5000) { // avgTemperature @5 SECONDS
          sendLoraMsg(27, avgTemperature);
          lastTimeAvgTemperature = millis();
        }
        if (millis() - lastTimeInternalTemperature > 5000) { // internalTemperature @5 SECONDS
          sendLoraMsg(28, internalTemperature);
          lastTimeInternalTemperature = millis();
        }
        
      }
      // ------------ MPPT ------------ //
      else if (__id == 513) {
        double MPPTInputVoltage = 0.01*extractDataNrBytes(__dta, 0, 2);
        double MPPTInputCurrent = 0.0005*extractDataNrBytes(__dta, 2, 2);
        double MPPTOutputVoltage = 0.01*extractDataNrBytes(__dta, 4, 2);
        double MPPTOutputCurrent = 0.0005*extractDataNrBytes(__dta, 6, 2);
        double MPPTOutputPower = MPPTOutputVoltage*MPPTOutputCurrent;

        if (millis() - lastTimeMPPTInputVoltage > 8000) { // MPPTInputVoltage @8 SECONDS
          sendLoraMsg(46, MPPTInputVoltage);
          lastTimeMPPTInputVoltage = millis();
        }
        if (millis() - lastTimeMPPTInputCurrent > 8000) { // MPPTInputCurrent @8 SECONDS
          sendLoraMsg(47, MPPTInputCurrent);
          lastTimeMPPTInputCurrent = millis();
        }
        if (millis() - lastTimeMPPTOutputVoltage > 5000) { // MPPTOutputVoltage @5 SECONDS
          sendLoraMsg(48, MPPTOutputVoltage);
          lastTimeMPPTOutputVoltage = millis();
        }
        if (millis() - lastTimeMPPTOutputCurrent > 5000) { // MPPTOutputCurrent @5 SECONDS
          sendLoraMsg(49, MPPTOutputCurrent);
          lastTimeMPPTOutputCurrent = millis();
        }
        if (millis() - lastTimeMPPTOutputPower > 5000) { // MPPTOutputPower @5 SECONDS
          sendLoraMsg(50, MPPTOutputPower);
          lastTimeMPPTOutputPower = millis();
        }
      }

    }
    */

  
  if (Serial.available() > 0) {
                    
          // Read the incoming data as a string
          String data = Serial.readStringUntil('\n');

          // Split the data by space
          int spaceIndex = data.indexOf(' ');
          if (spaceIndex != -1) {
            // Extract the ID and vehicle velocity as separate strings
            String idStr = data.substring(0, spaceIndex);
            String dataStr = data.substring(spaceIndex + 1);

            // Convert the strings to integers
            int id = idStr.toInt();
            if (id == 1) { // MC - Vehicle Velocity
              double vehicleVelocity = dataStr.toDouble();
            }
            else if (id == 2) { // MC - Heatsink Temp
              double heatsinkTemp = dataStr.toDouble();
              sendLoraMsg(2, heatsinkTemp);
            }
            else if (id == 3) { // MC - Motor Temp
              double motorTemp = dataStr.toDouble();
              sendLoraMsg(3, motorTemp);
            }
            else if (id == 4) { // HIGH_TEMP (THERMISTOR)
              double highTemperature = dataStr.toDouble();
              sendLoraMsg(4, highTemperature);
            }
            else if (id == 5) { // LOW_TEMP (THERMISTOR)
              double lowTemperature = dataStr.toDouble();
              sendLoraMsg(5, lowTemperature);
            }

            
            //0x600 [PACK_CURRENT, PACK_OPEN_VOLTAGE, CURRENT_LIMIT_STATUS, AVG_CURRENT, LOW_CELL_VOLTAGE, HIGH_CELL_VOLTAGE, INPUT_SUPPLY_VOLTAGE, RELAY STATE]
            //0x601 [INTAKE_TEMP, INTERNAL_TEMP, HIGH_TEMP, LOW_TEMP, AVG_TEMP, PACK_AMP_HOURS, HIGH_THERMISTOR_ID, LOW_THERMISTOR_ID]
            ///Serial.println(vehicleVelocity);
          }
          Serial.flush()
      }
        
        

  //sendLoraMsg(5, 1337);
  //RAKLoRa.rk_sendP2PData(1, "10", "abcd0f");
  //RAKLoRa.rk_sendP2PData(1, "10", "AABB");
}


