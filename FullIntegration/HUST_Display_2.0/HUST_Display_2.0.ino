#include "Platform.h"
#include "App_Common.h"
#include "ScreenConfiguration.h"
#include "buffer.h"
#include "Serial_CAN_FD.h"
#include "VehicleController.h"
#include "CANDecoder.h"
#include <SoftwareSerial.h>

/* DEFINING BUTTON PINS */
#define gas A2
#define brk A1
#define gear A0

#define L1 4
#define L2 A6
#define L3 A7

#define R1 A5
#define R2 A4
#define R3 A3

#define B1 9 
#define B2 5
#define B3 7
#define B4 6


/* +++++ CAN VARIABLES +++++ */
#define can_tx  2           // tx of serial can module connect to D2
#define can_rx  3           // rx of serial can module connect to D3

SoftwareSerial can_serial(can_tx, can_rx);

#define uart_can can_serial

unsigned long __id = 0;
unsigned char __ext = 0; // extended frame or standard frame
unsigned char __rtr = 0; // remote frame or data frame
unsigned char __fdf = 0; // can fd or can 2.0
unsigned char __len = 0; // data length
unsigned char __dta[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // data
/* ----- CAN VARIABLES ----- */


/* Global used for buffer optimization */
Gpu_Hal_Context_t host, *phost;
static byte rgb_yellow[] = {230, 230, 50}; // Yellow color 
static byte rbg_red[] = {250, 20, 1}; // Red color 
static byte rgb_white[] = {250, 250, 250}; // White color


// ECU - Electrical Control Unit (All logic for how driving works)
VehicleController ECU;


// Flags and Modes
bool inStartScreen = true;
bool high_beam_flag = false;
bool high_voltage_flag = true;
bool battery_error_flag = true;
bool mc_error_flag = true;
bool solar_error_flag = true;
bool driving_mode_error_flag = true;
bool error_flag = true;
bool eco_or_race_mode_flag = true; // Eco mode (0), Race mode (1)
bool cruise_control_flag = false;
bool warningLights = false;

// Screen Data
String global_data = "";

// Circular Buffers
CircularBuffer gas_buffer(10);
CircularBuffer break_buffer(10);

// Vehicle Parameters
int driving_mode_counter = 2; // 0: Neutral, 1: Drive, 2: Reverse
int max_gas = 1;
int min_gas = 2000;
int max_break = 0;
int min_break = 2000;
int current = 0;
int voltage_diff = 20;
int battery_voltage_max = 100;
int mc_watt = 20;
int mc_watt_max = 100;
int solar_watt = 20;
int solar_watt_max = 100;
int gas_potential;
int break_potential;
int last_brake_potential = 1 / 1337;
int cruise_control_velocity = 100;


/* ++++++++++++++++++++++ CAN ++++++++++++++++++++++ */

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

void clearCANDataArray() {
  for(int i = 0; i < 8; i++) {
    __dta[i] = 0;
  }
}

/* ----------------------- CAN ----------------------- */

/* ++++++++++++++++++++++ SCREEN GUI ++++++++++++++++++++++ */


void start_screen() {
  char maxgas_char[20];
  char mingas_char[20];
  char maxbreak_char[20];
  char minbreak_char[20];
  char current_char[20];

  sprintf(maxgas_char, "Max:%s", String(max_gas).c_str());
  sprintf(mingas_char, "Min:%s", String(min_gas).c_str());
  sprintf(maxbreak_char, "Max:%s", String(max_break).c_str());
  sprintf(minbreak_char, "Min:%s", String(min_break).c_str());
  sprintf(current_char, "Current:%s", String(current).c_str());

  Start_Set_Display(phost);

  // ---- Set boxes for text --------- //
  draw_rect(phost, 10, 60, 200, 100, 70, 200, 200);
  draw_rect(phost, 280, 60, 470, 100, 70, 200, 200);
  draw_rect(phost, 10, 130, 200, 170, 70, 200, 200);
  draw_rect(phost, 280, 130, 470, 170, 70, 200, 200);
  //draw_rect(phost, 140, 200, 340, 230, 70, 200, 200);

  //--------- Input text to screen ------------- //
  Write_Text(phost, 195, 0, 30, "HUST");
  Write_Text(phost, 50, 10, 25, "Gas");
  Write_Text(phost, 360, 10, 25, "Break");
  Write_Text(phost, 10, 65, 25, maxgas_char);
  Write_Text(phost, 10, 135, 25, mingas_char);
  Write_Text(phost, 280, 65, 25, maxbreak_char);
  Write_Text(phost, 280, 135, 25, minbreak_char);
  //Write_Text(phost, 140, 195, 25, current_char);


  Finish_Display(phost);
  delay(5);
}

void main_screen() {
  Start_Set_Display(phost);

  static double MPH_CONSTANT = 2.23694;
  String currentVelocity = String((int)vehicleVelocity*MPH_CONSTANT) + " MPH";
  Write_Text(phost, 195, 150, 30, currentVelocity.c_str());

  Finish_Display(phost);
  delay(5);
}

/*
//Function for main screen
void main_screen() {
    
    char velocity_char[25];
    char voltage_diff_char[20];
    char current_char[20];


    sprintf(velocity_char, "%s [mph]", String((int)vehicleVelocity).c_str());
    sprintf(voltage_diff_char, "Volt diff:%s", String((int)voltage_diff).c_str());
    sprintf(current_char, "Current:%s", String((int)current).c_str());

    Start_Set_Display(phost);
    //draw_rect(phost, 120 , 35, 300, 75); 
    //Write_Text(phost, 195, 0, 30, "Hust");
    Write_Text(phost, 210, 80, 31, String((int)vehicleVelocity).c_str());
    Write_Text(phost, 190, 115, 30, "mph");
    //Write_Text(phost, 10, 80, 22, voltage_diff_char);
    //Write_Text(phost, 10, 100, 22, current_char);
    //insert_line(phost, 440, 470, 10, 262);
    //insert_charging(phost, 0.7);


    Finish_Display(phost);
}
*/

/* ------------------- SCREEN GUI ----------------------- */


/* ++++++++++++++++++++++ BUTTON INPUTS ++++++++++++++++++++++ */

void checkSwitchButtonMainStartScreen() {

  static unsigned long lastTimeButtonPress = millis();
  byte switchScreenButton = digitalRead(B2);
  if(switchScreenButton == HIGH) {
    if (millis() - lastTimeButtonPress > 400) inStartScreen = !inStartScreen;
    //Serial.println("Screen changing");
    lastTimeButtonPress = millis();
  }
}

void checkGearModeButton() {
  int drivingModePotential = analogRead(gear);
  //Serial.println("[checkGearModeButton] driving potential: " + String(drivingModePotential));
  if (drivingModePotential < 341) { 
    ECU.drivingMode = 0; // NEUTRAL
  }
  else if(341 <= drivingModePotential && drivingModePotential < 682) { 
    ECU.drivingMode = 1; // DRIVE
  }
  else if (682 <= drivingModePotential) {
    ECU.drivingMode = 2; // REVERSE
  }
}

void checkEnterCruise() {
  static unsigned long lastTimeCruiseButtonPress = millis();
  byte toggleCruiseButton = digitalRead(L2);
  if(toggleCruiseButton == HIGH && millis() - lastTimeCruiseButtonPress > 500) {
    if (millis() - lastTimeCruiseButtonPress > 500) ECU.inCruiseControl = !ECU.inCruiseControl;
    //Serial.println("[checkEnterCruise] Toggling cruise, inCruise: " + String(ECU.inCruiseControl));
    lastTimeCruiseButtonPress = millis();
  }
}

void checkEnterRaceOrECOButton() {
  static unsigned long lastTimeModeButtonPress = millis();
  byte toggleModeButton = digitalRead(B4);
  if(toggleModeButton == HIGH) {
    if (millis() - lastTimeModeButtonPress > 500) ECU.inECO = !ECU.inECO;
    //Serial.println("[checkEnterRaceOrECOButton] Toggling mode (ECO/RACE), inECO: " + String(ECU.inECO));
    lastTimeModeButtonPress = millis();
  }
}

void checkLightButtonCommands() {

  /* Left Blinker (CAN ID: 0x010) */
  static bool leftBlinkerOn = false;
  static unsigned long lastTimeLeftBlinkerButtonPress = millis();
  byte leftBlinkerButton = digitalRead(L1);
  if (leftBlinkerButton == HIGH) {
    if (millis() - lastTimeLeftBlinkerButtonPress > 500) leftBlinkerOn = !leftBlinkerOn;
    //Serial.println("[checkLightButtonCommands] Toggling left blinker");
    //unsigned char __new_dta[1] = {(unsigned char)leftBlinkerOn};      // data
    __dta[0] = leftBlinkerOn;
    can_send(0x010, 0, 0, 0, 8, __dta);
    lastTimeLeftBlinkerButtonPress = millis();
  }

  /* Right Blinker (CAN ID: 0x011) */
  static bool rightBlinkerOn = false;
  static unsigned long lastTimeRightBlinkerButtonPress = millis();
  byte rightBlinkerButton = digitalRead(R1);
  if (rightBlinkerButton == HIGH) {
    if (millis() - lastTimeRightBlinkerButtonPress > 500) rightBlinkerOn = !rightBlinkerOn;
    //Serial.println("[checkLightButtonCommands] Toggling right blinker");
    __dta[0] = rightBlinkerOn;
    can_send(0x011, 0, 0, 0, 8, __dta);
    lastTimeRightBlinkerButtonPress = millis();
  }

  
  /* Warning Lights (CAN ID: 0x012) */
  static bool warningLightsOn = false;
  static unsigned long lastTimeWarningLightsButtonPress = millis();
  byte warningLightsButton = digitalRead(B3);
  if (warningLightsButton == HIGH) {
    if (millis() - lastTimeWarningLightsButtonPress > 500) warningLightsOn = !warningLightsOn;
    //Serial.println("[checkLightButtonCommands] Toggling warning lights");
    __dta[0] = warningLightsOn;
    can_send(0x012, 0, 0, 0, 8, __dta);
    lastTimeWarningLightsButtonPress = millis();
  }

}

void checkHornButtonCommand() {

  /* Horn (CAN ID: 0x020) */
  static unsigned long lastTimeHornButtonPress = millis();
  byte hornButton = digitalRead(B1);
  if (hornButton == HIGH && millis() - lastTimeHornButtonPress > 200) {
    //Serial.println("[checkHornButtonCommand] Beeping horn");
    __dta[0] = 1;
    can_send(0x020, 0, 0, 0, 8, __dta);
    lastTimeHornButtonPress = millis();
  }
}

void checkButtonsIncreaseDecreaseCruiseSpeed() {

  /* Increase Cruise Speed */
  static unsigned long lastTimeIncreaseButtonPress = millis();
  byte increaseButton = digitalRead(R2);
  if (increaseButton == HIGH && millis() - lastTimeIncreaseButtonPress > 700) {
    Serial.println("[checkButtonsIncreaseDecreaseCruiseSpeed] Increasing cruise speed");
    ECU.IncreaseCruiseSpeed();
    lastTimeIncreaseButtonPress = millis();
  }

  /* Decrease Cruise Speed (CAN ID: 0x031) */
  static unsigned long lastTimeDecreaseButtonPress = millis();
  byte decreaseButton = digitalRead(R3);
  if (decreaseButton == HIGH && millis() - lastTimeDecreaseButtonPress > 700) {
    //Serial.println("[checkButtonsIncreaseDecreaseCruiseSpeed] Decreasing cruise speed");
    ECU.decreaseCruiseSpeed();
    lastTimeDecreaseButtonPress = millis();
  }


}

/* ------------------- BUTTON INPUTS ----------------------- */

int calculateNewMaxPotentials(float potential, float min, float max) {
  int mappedValue = map(potential, 0, 1023, min, max);
  //Serial.println(mappedValue/6);
  return int(mappedValue/5);
}


void setup() {
  phost = &host;
  /* Init HW Hal */
  App_Common_Init(&host);

  pinMode(gas, INPUT); // RIGHT Potentiometer (ANALOG)
  pinMode(brk, INPUT); // LEFT Potentiometer (ANALOG)
  pinMode(gear, INPUT); // A 3-way switch (ANALOG)

  pinMode(L1, INPUT); // Left BLINKER (DIGITAL)
  pinMode(L2, INPUT); // Enter Cruise (DIGITAL)
  pinMode(L3, INPUT); 

  pinMode(R1, INPUT); // Right BLINKER (DIGITAL)
  pinMode(R2, INPUT); // INCREASE Cruise Speed (DIGITAL)
  pinMode(R3, INPUT); // DECREASE Cruise Speed (DIGITAL)

  pinMode(B1, INPUT); // Horn (DIGITAL)
  pinMode(B2, INPUT); // Switch between main- & start-screen (DIGITAL)
  pinMode(B3, INPUT); // Warning lights (DIGITAL)
  pinMode(B4, INPUT); // Change driving mode ECO/RACE - (DIGITAL)

  Serial.begin(9600);

  /* INITIALIZE CAN*/
  uart_init(9600);
  can_speed_20(500000); // set can bus baudrate to 500k
}


void loop() {

  int gasPotential = abs(analogRead(A1) - 1337);
  int brakePotential = analogRead(A2);

  checkSwitchButtonMainStartScreen();

  if (inStartScreen) {
    static byte meanGasBrakeCounter = 0;
    gas_buffer.add(gasPotential);
    break_buffer.add(brakePotential);
    meanGasBrakeCounter++;

  /* calc new mean for gas IF we have 10 potential values */
  if (meanGasBrakeCounter > 10) {
    int mean_gas = gas_buffer.get_mean(); // -1337 since potential is reversed
    int mean_brake = break_buffer.get_mean();
    
    /* Setting max || min GAS */
    if (mean_gas > max_gas) {
      max_gas = mean_gas;
      ECU.Max_gas_N_reverse_potential = mean_gas;
    }
    else if (mean_gas < min_gas) {
      min_gas = mean_gas;
      ECU.Min_gas_N_reverse_potential = mean_gas;
    }

    /* Setting max || min BRAKE */
    if (mean_brake > max_break) {
      max_break = mean_brake;
      ECU.Max_brake_potential = mean_brake;
    }
    else if (mean_brake < min_break) {
      min_break = mean_brake;
      ECU.Min_brake_potential = mean_brake;
    }

    meanGasBrakeCounter = 0;
  }

    start_screen(); // Show start_screen GUI
  }
  else {

    if(read_can(&__id, &__ext, &__rtr, &__fdf, &__len, __dta)) {
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

        /*
        ** @DecodeCANMsg
        ** Decodes CAN msg and updates the corresponding
        ** variable in "GlobalVariableReceive.h"
        */
        DecodeCANMsg(__id, __dta); 
      }

        main_screen(); // Show main_screen GUI

        checkLightButtonCommands(); // Check if light buttons are pressed
        checkHornButtonCommand(); // Check if horn button is pressed
        checkEnterRaceOrECOButton(); // Check if race/eco button is pressed
        checkGearModeButton(); // Check if driving mode (neutral/reverse/drive) is rotated

        if (ECU.inCruiseControl) { 
          checkButtonsIncreaseDecreaseCruiseSpeed(); // Check if should increase/decrease cruise speed
        }

        ECU.vehicleControlLoop(gasPotential, brakePotential);
        

    }
}
