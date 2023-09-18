#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"

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


int gasPotential = 0;
int breakPotential = 0;
int pot_counter = 0;
int drivingMode = 0; /* 0 = Neutral, 1 = Drive, 2 = Reverse */
int cruiseControlVelocity = 100;


bool inStartScreen = true;
bool data_received_flag = false;
bool update_screen_flag = true;
bool start_screen_flag = true;
bool main_screen_flag = false;
bool high_beam_flag = false;
bool high_voltage_flag = true;
bool battery_error_flag = true;
bool mc_error_flag = true;
bool solar_error_flag = true;
bool driving_mode_error_flag = true;
bool error_flag = true;

bool inCruiseControl = false;
bool inEcoOrRace = true; /*--- Eco 0 Race 1 ---*/
bool activateRightBlinker = false;
bool activateLeftBlinker = false;
bool activateHazardLights = false;
bool activateHighBeam = false;


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


void setup() {
  /* SETUP OF CAN MODULE */
  Serial.begin(9600);
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
        
        int CAN_data = atoi(__dta);
        switch __id {
          case 0x300: gas_potential = CAN_data break;
          case 0x301: break_potential = CAN_data break;
          case 0x302: drivingMode = CAN_data break;
          case 0x303: isCruising = CAN_data break;
          case 0x304: inEcoOrRace = CAN_data break;
          case 0x305: inStartScreen = CAN_data break;
          case 0x306: break;
          case 0x307: activateRightBlinker = CAN_data break;
          case 0x308: activateLeftBlinker = CAN_data break;
          case 0x309: activateHazardLights = CAN_data break;
          case 0x30a: activateHighBeam = CAN_data break;
          case 0x30b: break;
          case 0x30c: break;
        }
        Serial.println(atoi(__dta));
    }
}