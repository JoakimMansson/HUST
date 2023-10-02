#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"
#include "VehicleController.h"
#include "CANDecoder.h"

// For debugging
#define DEBUG 1

#if DEBUG == 1
#define debugln(x) Serial.println(x)
#define debug(x) Serial.print(x)
#else
#define debugln(x)
#define debug(x)
#endif

#define can_tx  8           // tx of serial can module connect to D2
#define can_rx  9           // rx of serial can module connect to D3

SoftwareSerial can_serial(can_tx, can_rx);

#define uart_can can_serial

#define left_blinker_pin 1
#define right_blinker_pin 2
#define horn_pin 3 
#define high_beam_pin 4 

// CAN Variables
unsigned long __id = 0;
unsigned char __ext = 0; // extended frame or standard frame
unsigned char __rtr = 0; // remote frame or data frame
unsigned char __fdf = 0; // can fd or can 2.0
unsigned char __len = 0; // data length
unsigned char __dta[8]; // data


// Light Variables
bool isRightBlinker = false;
bool isLeftBlinker = false;
bool isWarningLights = false;

VehicleController ECU;


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

/* +++++++++++++++ LIGHTS & HORN +++++++++++++++ */

void highBeam(int toggleHighBeam) {
  //digitalWrite(high_beam_pin, toggleHighBeam);
}

void blink() {
  static unsigned long lastBlink = millis();
  static bool wasLastOn = false;

  if (millis() - lastBlink >= 1000) {

    if (isWarningLights) {
      wasLastOn = !wasLastOn;
      //digitalWrite(left_blinker_pin, wasLastOn);
      //digitalWrite(right_blinker_pin, wasLastOn);
      //lastBlink = millis();
    } 
    else if (isLeftBlinker) {
      wasLastOn = !wasLastOn;
      //digitalWrite(left_blinker_pin, wasLastOn);
      lastBlink = millis();
    } 
    else if (isRightBlinker) {
      wasLastOn = !wasLastOn;
      //digitalWrite(left_blinker_pin, wasLastOn);
      lastBlink = millis();
    }

  }
}


void beep() {
  //digitalWrite(horn_pin, HIGH);
}

/* ------------- LIGHTS & HORN ------------- */

void setup() 
{
  Serial.begin(9600);

  /* 
  * IMPORTANT! DEFINE THE CORRECT DIGITAL PINS BELOW
  */
  //pinMode(left_blinker_pin, OUTPUT); // Left blinker
  //pinMode(right_blinker_pin, OUTPUT); // Right blinker
  //pinMode(horn_pin, OUTPUT); //


  /* SETUP OF CAN MODULE */
  uart_init(9600);
  can_speed_20(500000);          // set can bus baudrate to 500k
}

void loop() 
{
  static bool inStartScreen = true;
  static int gasPotential = 0;
  static int brakePotential = 0;

  if(read_can(&__id, &__ext, &__rtr, &__fdf, &__len, __dta))
    {
      /*
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
        */
        

        //Serial.println("ID: 0x" + String(__id) + ", DATA: " + String(__dta[0]));
        /* LEFT BLINKER */
        if (__id == 0x010) { // Left blinker
            int toggle = atoi(__dta);
            if(toggle == 1) {
              isLeftBlinker = true;
              isRightBlinker = false;
            }
            else {
              isLeftBlinker = false;
            }
        }
        /* RIGHT BLINKER */
        else if (__id == 0x011) { // Right blinker
            int toggle = atoi(__dta);
            if(toggle == 1) {
              isLeftBlinker = false;
              isRightBlinker = true;
            }
            else {
              isRightBlinker = false;
            }
        } 
        /* WARNING LIGHTS */
        else if (__id == 0x012) { // Warning lights
          if(atoi(__dta) == 1) {
              isLeftBlinker = false;
              isRightBlinker = false;
              isWarningLights = true;
            }
            else {
              isWarningLights = false;
            }
        }
        /* HIGH BEAM */
        else if (__id == 0x016) { // High beam
          highBeam(atoi(__dta));
        }
        /* HORN */
        else if (__id == 0x020) { // Horn 
          beep();
        }
        /* GAS POTENTIAL */
        else if (__id == 0x050) { // Gas potential
          long extractedPotential;
          memcpy(&extractedPotential, __dta, sizeof(long));
          extractedPotential -= 8; // Offset
          if(extractedPotential < 0) extractedPotential = 0;
          if(extractedPotential > 1023) extractedPotential = 1023;
          Serial.println("GasPot (long): " + String(extractedPotential));
        }
        /* BRAKE POTENTIAL */
        else if (__id == 0x051) { // Brake potential
          long extractedPotential;
          memcpy(&extractedPotential, __dta, sizeof(long));
          extractedPotential -= 8; // Offset
          if(extractedPotential < 0) extractedPotential = 0;
          if(extractedPotential > 1023) extractedPotential = 1023;
          Serial.println("BrakePot (long): " + String(extractedPotential));
        }
        /* DRIVING MODE */
        else if (__id == 0x013) { // Driving mode
          ECU.drivingMode = __dta[0];
        }
        /* CRUISING MODE */
        else if (__id == 0x014) { // In cruising
          ECU.inCruising = __dta[0];
        }
        /* INC/DEC CRUISE SPEED */
        else if (__id == 0x017) { // Inc Dec Cruise Speed
          if (__dta[0] == 0) ECU.decreaseCruiseSpeed();
          else ECU.IncreaseCruiseSpeed(); 
        }
        /* IN START SCREEN */
        else if (__id == 0x030) { // In start screen
          inStartScreen = __dta[0];
        }
        else {
          DecodeCANMsg(__id, __dta);
        }

    }
    
  blink();
  if (!inStartScreen) {
    //ECU.vehicleControlLoop(gasPotential, brakePotential);
  }

  
}


