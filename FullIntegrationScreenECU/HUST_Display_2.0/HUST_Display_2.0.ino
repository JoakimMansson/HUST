#include "Platform.h"
#include "App_Common.h"
//#include "ScreenConfiguration.h"
#include "Functions.h"
#include "buffer.h"
#include "VehicleController.h"
//#include "CANDecoder.h"
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
bool inStartScreen = true; // If current screen is start
bool potentialsSet = false; // If gas and brake potentials has been set
bool highBeamActive = false; // If high beam is active
bool leftBlinkerActive = false; // If left blinker is active
bool rightBlinkerActive = false; // If right blinker active
bool warningLightsActive = false; // If warning lights active
bool halfLightActive = false; // If half lights active

// Screen Data
String global_data = "";

// Circular Buffers
CircularBuffer gas_buffer(10);
CircularBuffer break_buffer(10);

// Vehicle Parameters
int drivingModeCounter = 0; // 0: Neutral, 1: Drive, 2: Reverse
int maxGas = 1;
int minGas = 2000;
int maxBrake = 0;
int minBrake = 2000;

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

bool connection_established_to_RPI = false;



/* ++++++++++++++++++++++ SCREEN GUI ++++++++++++++++++++++ */


void start_screen() {
  char maxgas_char[20];
  char mingas_char[20];
  char maxbreak_char[20];
  char minbreak_char[20];
  char current_char[20];

  sprintf(maxgas_char, "Max:%s", String(maxGas).c_str());
  sprintf(mingas_char, "Min:%s", String(minGas).c_str());
  sprintf(maxbreak_char, "Max:%s", String(maxBrake).c_str());
  sprintf(minbreak_char, "Min:%s", String(minBrake).c_str());
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
  Write_Text(phost, 360, 10, 25, "Gas");
  Write_Text(phost, 50, 10, 25, "Brake");
  Write_Text(phost, 10, 65, 25, maxbreak_char);
  Write_Text(phost, 10, 135, 25, minbreak_char);
  Write_Text(phost, 280, 65, 25, maxgas_char);
  Write_Text(phost, 280, 135, 25, mingas_char);

  static unsigned long lastTimeSetPotentialsBlink = millis();
  if (millis() - lastTimeSetPotentialsBlink > 300 && !potentialsSet) {

    if (millis() - lastTimeSetPotentialsBlink > 1000) lastTimeSetPotentialsBlink = millis();
    Write_Text_Color(phost, 135, 215, 16, "SET GAS &  BRAKE", 255, 1, 1);
  }

  if (halfLightActive) {
    solar_cell_icon(phost, 1, 1, 255);
  }
  
  Finish_Display(phost);
}

void main_screen() {


  //sprintf(motor_temp_char, "Motor T:%s", String((motorTemp.toInt()).c_str());
  //sprintf(heatsink_temp_char, "Current:%s", String(heatsinkTemp.toInt()).c_str());

  Start_Set_Display(phost);

  //Write_Text(phost, 300, 125, 31, String(millis() - ECU.lastTimeBusCurrentReceived).c_str());
  //Write_Text(phost, 300, 160, 31, String(busCurrent).c_str());

  // ++++++ Velocity ++++++ //
  Write_Text(phost, 225, 125, 31, String(int(vehicleVelocity)).c_str());
  // ----- Velocity ----- //

  // ++++++ MC DATA ++++++ //
  char motor_temp[25];
  char heatsink_temp[25];

  sprintf(motor_temp, "Motor:%s", String(motorTemp).c_str());
  sprintf(heatsink_temp, "Heatsink:%s", String(heatsinkTemp).c_str());

  Write_Text(phost, 350, 10, 30, "MC");
  Write_Text(phost, 300, 50, 22, motor_temp);
  Write_Text(phost, 300, 70, 22, heatsink_temp);
  // ----- MC DATA ----- //


  // ++++++ BMS DATA ++++++ //
  char internal_temp[20];
  char high_temp[20];
  char low_temp[20];
  sprintf(internal_temp, "BMS:%s", String(internalTemperature).c_str());
  sprintf(high_temp, "High:%s", String(highTemperature).c_str());
  //sprintf(low_temp, "Low:%s", String(lowTemperature).c_str());

  Write_Text(phost, 20, 10, 30, "BMS");
  Write_Text(phost, 20, 50, 22, internal_temp);
  Write_Text(phost, 20, 70, 22, high_temp);
  //Write_Text(phost, 20, 90, 22, low_temp);
  // ----- BMS DATA ----- //

  // ++++++ WARNING TEXT FOR TEMP ++++++ //
  static unsigned long lastWarningTempTextBlink = 0;
  bool tempWarning = highTemperature > 50 || motorTemp > 100 || heatsinkTemp > 60 || internalTemperature > 80;
  if (millis() - lastWarningTempTextBlink > 300 && tempWarning) {

    if (millis() - lastWarningTempTextBlink > 1000) lastWarningTempTextBlink = millis();
    Write_Text_Color(phost, 165, 95, 16, "CHECK TEMP", 255, 1, 1);
  }

  // ----- WARNING TEXT FOR TEMP ----- //

  // ++++++ UI FOR LIGHTS ++++++ //
  static unsigned long lastLightBlink = 0;
  static const unsigned long blinkInterval = 500; // 500 ms interval

  if (millis() - lastLightBlink >= blinkInterval) {

    if (millis() - lastLightBlink >= 1000) lastLightBlink = millis(); // Update the last blink time


    if (leftBlinkerActive) {
      Write_Text_Color(phost, 200, 50, 60, "<", 1, 1, 255);
    } 
    else if (rightBlinkerActive) {
      Write_Text_Color(phost, 250, 50, 60, ">", 1, 1, 255);
    } 
    else if (warningLightsActive) {
      Write_Text_Color(phost, 220, 50, 60, "!", 1, 1, 255);
    }
  }

  if (halfLightActive) {
    solar_cell_icon(phost, 1, 1, 255);
  }
  // ----- UI FOR LIGHTS ----- //


  // ++++++ SHOWING DRIVINGMODES ++++++ //
  String letter = "";
  switch (ECU.drivingMode) {
    case 0: letter="N"; break;
    case 1: letter="D"; break;
    case 2: letter="R"; break;
  }

  Write_Text(phost, 225, 170, 31, letter.c_str());
  Write_Text(phost, 210, 230, 23, "Mode");
  insert_line(phost, 205, 275, 215, 265, 20);
  // ----- SHOWING DRIVINGMODES ----- //
  

  // ++++++ SHOWING ICONS ++++++ //
  if(ECU.inCruiseControl) {
    //char cruise[20];
    //sprintf(cruise, "Cruise:%s", String(int(ECU.velocityCruiseControl)).c_str());

    Write_Text(phost, 320, 150, 24, "Cruise");
    Write_Text(phost, 360, 175, 24, String(ECU.velocityCruiseControl).c_str());
  }

  if(highBeamActive) {
    high_beam(phost);   
  }
  // ----- SHOWING ICONS ----- //

    
  Finish_Display(phost);
}

/* ------------------- SCREEN GUI ----------------------- */


/* ++++++++++++++++++++++ BUTTON INPUTS ++++++++++++++++++++++ */

void checkSwitchButtonMainStartScreen() {

  static unsigned long lastTimeButtonPress = millis();
  byte switchScreenButton = digitalRead(B2);
  if(switchScreenButton == HIGH) {
    if (millis() - lastTimeButtonPress > 400) {
      bool wasInStartScreen = inStartScreen;
      inStartScreen = !inStartScreen;
    
      if (!wasInStartScreen && inStartScreen) { // RESET POTENTIALS IF GOING FROM MAIN SCREEN TO START SCREEN
        maxGas = 0;
        minGas = 2000;
        maxBrake = 0;
        minBrake = 2000;
      }
    } 
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
  else if(341 <= drivingModePotential && drivingModePotential < 682 && !(ECU.drivingMode == 2 && abs(vehicleVelocity) > 1)) { 
    ECU.drivingMode = 1; // DRIVE
  }
  else if (drivingModePotential >= 682 && !(ECU.drivingMode == 1 && abs(vehicleVelocity) > 1)) {
    ECU.drivingMode = 2; // REVERSE
  }
}

void checkEnterCruise() {
  static unsigned long lastTimeCruiseButtonPress = millis();
  byte toggleCruiseButton = digitalRead(L2);
  bool isInDrive = ECU.drivingMode != 2 && ECU.drivingMode != 0;
  if(toggleCruiseButton == HIGH && isInDrive) {
    if (millis() - lastTimeCruiseButtonPress > 500) ECU.inCruiseControl = !ECU.inCruiseControl;
    
    if (ECU.inCruiseControl) ECU.velocityCruiseControl = vehicleVelocity;
    else ECU.velocityCruiseControl = 0;
    //Serial.println("[checkEnterCruise] Toggling cruise, inCruise: " + String(ECU.inCruiseControl));
    lastTimeCruiseButtonPress = millis();
  }
}

/*
void checkEnterRaceOrECOButton() {
  static unsigned long lastTimeModeButtonPress = millis();
  byte toggleModeButton = digitalRead(B4);
  if(toggleModeButton == HIGH) {
    if (millis() - lastTimeModeButtonPress > 500) ECU.inECO = !ECU.inECO;
    //Serial.println("[checkEnterRaceOrECOButton] Toggling mode (ECO/RACE), inECO: " + String(ECU.inECO));
    lastTimeModeButtonPress = millis();
  }
}
*/

void checkLightButtonCommands() {

  /* Left Blinker (CAN ID: 0x010) */
  static unsigned long lastTimeLeftBlinkerButtonPress = millis();
  byte leftBlinkerButton = digitalRead(L1);
  if (leftBlinkerButton == HIGH) {
    if (millis() - lastTimeLeftBlinkerButtonPress > 500) leftBlinkerActive = !leftBlinkerActive;
    //Serial.println("[checkLightButtonCommands] Toggling left blinker");
    //unsigned char __new_dta[1] = {(unsigned char)leftBlinkerOn};      // data
    Serial.println("10 " + String(leftBlinkerActive));
    
    if (leftBlinkerActive) {
      rightBlinkerActive = false;
      warningLightsActive = false;
    }
    else {
      leftBlinkerActive = false;
    }
    lastTimeLeftBlinkerButtonPress = millis();
  }

  /* Right Blinker (CAN ID: 0x011) */
  static unsigned long lastTimeRightBlinkerButtonPress = millis();
  byte rightBlinkerButton = digitalRead(R1);
  if (rightBlinkerButton == HIGH) {
    if (millis() - lastTimeRightBlinkerButtonPress > 500) rightBlinkerActive = !rightBlinkerActive;
    //Serial.println("[checkLightButtonCommands] Toggling right blinker");
    Serial.println("11 " + String(rightBlinkerActive));
    
    if (rightBlinkerActive) {
      leftBlinkerActive = false;
      warningLightsActive = false;
    }
    else {
      rightBlinkerActive = false;
    }
    lastTimeRightBlinkerButtonPress = millis();
  }

  
  /* Warning Lights (CAN ID: 0x012) */
  static unsigned long lastTimeWarningLightsButtonPress = millis();
  byte warningLightsButton = digitalRead(B3);
  if (warningLightsButton == HIGH) {
    if (millis() - lastTimeWarningLightsButtonPress > 500) warningLightsActive = !warningLightsActive;
    //Serial.println("[checkLightButtonCommands] Toggling warning lights");
    Serial.println("12 " + String(warningLightsActive));
    
    if (warningLightsActive) {
      leftBlinkerActive = false;
      rightBlinkerActive = false;
    }
    else {
      warningLightsActive = false;
    }

    lastTimeWarningLightsButtonPress = millis();
  }
}

void checkHornButtonCommand() {

  /* Horn (CAN ID: 0x020) */
  static unsigned long lastTimeHornButtonPress = millis();
  byte hornButton = digitalRead(B1);
  if (hornButton == HIGH && millis() - lastTimeHornButtonPress > 100) {
    //Serial.println("[checkHornButtonCommand] Beeping horn");
    Serial.println("13 1");
    lastTimeHornButtonPress = millis();
  }
}

void checkActivateHighBeam() {
  static bool highBeamOn = false;
  static unsigned long lastTimeHighBeamButtonPress = millis();
  static unsigned long sumTimeHighBeamButtonPress = millis();
  byte highBeamButton = digitalRead(L3);

  if (highBeamButton == HIGH && millis() - lastTimeHighBeamButtonPress > 700) {
    highBeamOn = !highBeamOn;
    //Serial.println("[checkLightButtonCommands] Toggling left blinker");
    //unsigned char __new_dta[1] = {(unsigned char)leftBlinkerOn};      // data
    Serial.println("14 " + String(highBeamOn));
    highBeamActive = highBeamOn;
    lastTimeHighBeamButtonPress = millis();
  }
  
  static unsigned long startTimeSumMillis = 0;
  if (highBeamButton == HIGH && startTimeSumMillis == 0) {
    startTimeSumMillis = millis();
  }
  else if (highBeamButton == LOW) {
    startTimeSumMillis = 0;
  }
  else if (millis() - startTimeSumMillis > 3000) {
    Serial.println("14 " + String(highBeamOn));
    delay(20);
  }

}

void checkActivateHalfLight() {
  /*
  static unsigned long lastTimeHalfLightButtonPress = millis();
  byte halfLightButton = digitalRead(B4);
  if (halfLightButton == HIGH && millis() - lastTimeHalfLightButtonPress > 10000) {
    halfLightActive = !halfLightActive;
    Serial.println("15 " + String(halfLightActive));
    
    lastTimeHalfLightButtonPress = millis();
  }
  */

  byte halfLightButton = digitalRead(B4);
  static unsigned long startTimeSumMillis = 0;
  static bool hasSwitched = false;
  if (halfLightButton == HIGH && startTimeSumMillis == 0) {
    startTimeSumMillis = millis();
  }
  else if (halfLightButton == LOW) {
    startTimeSumMillis = 0;
    hasSwitched = false;
  }
  else if (millis() - startTimeSumMillis > 3000) {
    if (!hasSwitched) {
      halfLightActive = !halfLightActive;
      hasSwitched = true;
    }
    startTimeSumMillis = 0;
    Serial.println("15 " + String(halfLightActive));
  }
  
}

void checkButtonsIncreaseDecreaseCruiseSpeed() {

  /* Increase Cruise Speed */
  static unsigned long lastTimeIncreaseButtonPress = millis();
  byte increaseButton = digitalRead(R2);
  if (increaseButton == HIGH && millis() - lastTimeIncreaseButtonPress > 1000) {
    //Serial.println("[checkButtonsIncreaseDecreaseCruiseSpeed] Increasing cruise speed");
    ECU.IncreaseCruiseSpeed();
    lastTimeIncreaseButtonPress = millis();
  }

  /* Decrease Cruise Speed (CAN ID: 0x031) */
  static unsigned long lastTimeDecreaseButtonPress = millis();
  byte decreaseButton = digitalRead(R3);
  if (decreaseButton == HIGH && millis() - lastTimeDecreaseButtonPress > 1000) {
    //Serial.println("[checkButtonsIncreaseDecreaseCruiseSpeed] Decreasing cruise speed");
    ECU.decreaseCruiseSpeed();
    lastTimeDecreaseButtonPress = millis();
  }

}

/* ------------------- BUTTON INPUTS ----------------------- */

/* ++++++++++++++++++++++ CHECK IF POTENTIALS SET ++++++++++++++++++++++ */

bool potentialsSetCheck() {
  if (maxBrake - minBrake < 100 || maxGas - minGas < 100) {
    potentialsSet = false;
    return false;
  }
  else {
    potentialsSet = true;
    return true; 
  } 
}

/* ------------------- CHECK IF POTENTIALS SET ----------------------- */

void insertionSort(int arr[], int size) {
  for (int i = 1; i < size; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
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
  pinMode(L3, INPUT); // High Beam (DIGITAL)

  pinMode(R1, INPUT); // Right BLINKER (DIGITAL)
  pinMode(R2, INPUT); // INCREASE Cruise Speed (DIGITAL)
  pinMode(R3, INPUT); // DECREASE Cruise Speed (DIGITAL)

  pinMode(B1, INPUT); // Horn (DIGITAL)
  pinMode(B2, INPUT); // Switch between main- & start-screen (DIGITAL)
  pinMode(B3, INPUT); // Warning lights (DIGITAL)
  pinMode(B4, INPUT); // NOW: Change half light (DIGITAL), BEFORE: Change driving mode ECO/RACE - (DIGITAL)

  Serial.begin(19200);

  unsigned long start_time = millis();
  while(millis() - start_time < 8000) {Serial.println("STARTING_SCREEN");}
}


void loop() {
  
  checkActivateHighBeam(); // Check if high beam should toggle
  checkActivateHalfLight(); // Check if half light should toggle
  checkLightButtonCommands(); // Check if light buttons are pressed
  checkHornButtonCommand(); // Check if horn button is pressed
  

  int meanGas = 0;
  int meanBrake = 0;

  int i = 0;
  while (i < 10) {
    int gasPotential = analogRead(A2);
    int brakePotential = abs(analogRead(A1) - 1337);

    if (brakePotential > 1000) brakePotential = 999; // For kabel som glappar

    gas_buffer.add(gasPotential);
    break_buffer.add(brakePotential);
    i++;
  }

    if (potentialsSetCheck() || !inStartScreen) {
      checkSwitchButtonMainStartScreen();
    }

  meanGas = gas_buffer.get_mean();
  meanBrake = break_buffer.get_mean();

  if (inStartScreen) {
    
    
    /* Setting max || min GAS */
    if (meanGas > maxGas) {
      maxGas = meanGas;
      ECU.Max_gas_N_reverse_potential = meanGas;
    }
    else if (meanGas < minGas) {
      minGas = meanGas;
      ECU.Min_gas_N_reverse_potential = meanGas;
    }

    /* Setting max || min BRAKE */
    if (meanBrake > maxBrake) {
      maxBrake = meanBrake;
      ECU.Max_brake_potential = meanBrake;
    }
    else if (meanBrake < minBrake) {
      minBrake = meanBrake;
      ECU.Min_brake_potential = meanBrake;
    }


    start_screen(); // Show start_screen GUI
  }
  else {
  
        checkEnterCruise();
        //checkEnterRaceOrECOButton(); // Check if race/eco button is pressed
        checkGearModeButton(); // Check if driving mode (neutral/reverse/drive) is rotated
        main_screen(); // Show main_screen GUI
        

        if (ECU.inCruiseControl) { 
          checkButtonsIncreaseDecreaseCruiseSpeed(); // Check if should increase/decrease cruise speed
        }
          
        static unsigned long lastSentPotential = millis();
        if (millis() - lastSentPotential >= 70) {
          ECU.vehicleControlLoop(meanGas, meanBrake);
          lastSentPotential = millis();
        }

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
            if (id == 0) {
              ECU.inCruiseControl = false;
              ECU.velocityCruiseControl = 0;
            }
            else if (id == 1) { // MC - Vehicle Velocity
              ECU.lastVehicleVelocity = vehicleVelocity;
              vehicleVelocity =  dataStr.toDouble();
            }
            else if (id == 2) { // MC - Heatsink Temp
              heatsinkTemp =  dataStr.toInt();
            }
            else if (id == 3) { // MC - Motor Temp
              motorTemp =  dataStr.toInt();
            }
            else if (id == 4) { // INTERNAL TEMP (THERMISTOR)
              internalTemperature =  dataStr.toInt();
            }
            else if (id == 5) { // HIGH TEMP (THERMISTOR)
              highTemperature =  dataStr.toInt();
            }
            else if (id == 6) { // LOW TEMP (THERMISTOR)
              lowTemperature = dataStr.toInt();
            }
            else if (id == 9) { // MC - BUS CURRENT
               busCurrent = dataStr.toDouble();
            }

            
            //0x600 [PACK_CURRENT, PACK_OPEN_VOLTAGE, CURRENT_LIMIT_STATUS, AVG_CURRENT, LOW_CELL_VOLTAGE, HIGH_CELL_VOLTAGE, INPUT_SUPPLY_VOLTAGE, RELAY STATE]
            //0x601 [INTAKE_TEMP, INTERNAL_TEMP, HIGH_TEMP, LOW_TEMP, AVG_TEMP, PACK_AMP_HOURS, HIGH_THERMISTOR_ID, LOW_THERMISTOR_ID]
            ///Serial.println(vehicleVelocity);
          }
        }
        
        
        main_screen(); // Show main_screen GUI
    }
}
