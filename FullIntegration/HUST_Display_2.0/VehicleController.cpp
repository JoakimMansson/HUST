#include "VehicleController.h"
#include "Serial_CAN_FD.h"
#include <Arduino.h>
#include <SoftwareSerial.h>



void VehicleController::vehicleControlLoop(int &gas_N_reverse_potential, int &brake_potential, int drivingMode, bool inCruiseControl, bool inECO, bool inStartScreen, int cruiseSpeedIncDec) {
  this->drivingMode = drivingMode;
  this->inCruiseControl = inCruiseControl;
  this->inECO = inECO;
  this->inStartScreen = inStartScreen;
  this->cruiseSpeedIncDec = cruiseSpeedIncDec;

  /*
  unsigned char CAN_available = !digitalRead(CAN_INT_PIN);
  if(CAN_available > 0){
    unsigned long start_time = millis();
    while (CAN_MSGAVAIL == CAN.checkReceive() && millis() - start_time < 120) {

        
        //READ CAN DATA
        

        // Extract vehicle velocity speed
        if(CAN_ID == "1027")
        {
          vehicleVelocity = extractBytesToDecimal(CAN_data, 4, 4);
          lastTimePointVelocityFetched = millis();
          //debugln("Vehicle velocity: " + String(vehicleVelocity));
        }
        // Extract bus_current
        if(CAN_ID == "1026")
        {
          busCurrent = extractBytesToDecimal(CAN_data, 4, 4);
          //debugln("Bus current: " + String(busCurrent));
          //delay(800);
        }
    }
  }
  */
  updateCurrentDrivingMode();
  enterCruiseControl();
  
  /*
  debugln("gas_N_reverse_potential before: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  */

  applyCruiseControl(gas_N_reverse_potential, brake_potential); // IF IN CRUISE CONTROL UPDATE gas_N_reverse_potential and brake_potential
  applyECOControl(gas_N_reverse_potential); // IF IN ECO UPDATE gas_N_reverse_potential

  // Sends drive commands if not in start screen
  if(!inStartScreen)
  {
    controlCar(gas_N_reverse_potential, brake_potential);
  }
  last_gas_N_reverse_potential = gas_N_reverse_potential;
  last_brake_potential = brake_potential;
  lastVehicleVelocity = vehicleVelocity; // Set last vehicle velocity to current velocity
  /*
  debugln("gas_N_reverse_potential after: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  debugln("Vehicle velocity: " + String(vehicleVelocity) + ", velocityCruiseControl: " + String(velocityCruiseControl));
  */
  lastCruiseSpeedIncDec = cruiseSpeedIncDec;
  lastInCruiseControl = inCruiseControl;
}

void VehicleController::decreaseCruiseSpeed() {
  cruiseSpeedIncDec -= 2;
}

void VehicleController::IncreaseCruiseSpeed() {
  cruiseSpeedIncDec += 2;
}


void VehicleController::IEEE754ToArray(unsigned char (&brake_drive_reverse)[8], String ieee754) {
    for (int i = 0; i < 4; i++) {
      if (i == 0) {
        String byte = ieee754.substring(0, 2);
        unsigned char unsignedByte = hexStringToInt(byte.c_str());
        brake_drive_reverse[4] = unsignedByte;
        //debugln("Byte " + String(i) + ": " + String(unsignedByte));
      } else {
        String byte = ieee754.substring(i * 2, i * 2 + 2);
        unsigned char unsignedByte = hexStringToInt(byte.c_str());
        brake_drive_reverse[4 + i] = unsignedByte;
        //debugln("Byte " + String(i) + ": " + String(unsignedByte));
      }
    }
}

String VehicleController::IEEE754(const double potential) {
  double f = static_cast<double>(potential);
  char *bytes = reinterpret_cast<char *>(&f);

  String ieee754 = "";
  for (int i = 0; i < 4; i++) {
    String tempByte = String(bytes[i], HEX);
    //debugln("Byte " + String(i) + ": " + tempByte);

    
    if (tempByte.length() > 2) { //Edgecase for when tempByte is ex: ffffc0 and we only need c0
      //debugln("Byte.length>2: " + tempByte);
      ieee754 += tempByte.substring(tempByte.length() - 2, tempByte.length() - 1) + tempByte.substring(tempByte.length() - 1, tempByte.length());
    } else {
      ieee754 += String(bytes[i], HEX);
      // debugln("Hex " + String(i) + ": " + String(bytes[i], HEX));  //4668
    }
  }

  int remainingZeros = 8 - ieee754.length(); // For when IEEE754.length() < 8
  for (int i = 0; i < remainingZeros; i++) {
    ieee754 = "0" + ieee754;
  }

  return ieee754;
}

double VehicleController::hexStringToInt(const char *hexString) {
  return (double)strtol(hexString, NULL, 16);
}

void VehicleController::resetArrays() {
  for(int i = 4; i < 8; i++)
  {
    BRAKE_ARR[i] = 0;
    DRIVE_ARR[i] = 0;
    REVERSE_ARR[i] = 0;
  }
}


void VehicleController::updateCurrentDrivingMode() {
  if (drivingMode == 1) {
    isDriving = true;

    isBraking = false;
    isReversing = false;
    isNeutral = false;
    return;
  } else if (drivingMode == 2) {
    isReversing = true;

    isDriving = false;
    isBraking = false;
    isNeutral = false;
    return;
  } else if (drivingMode == 0) {
    isNeutral = true;

    isBraking = false;
    isDriving = false;
    isReversing = false;
    return;
  }
}


void VehicleController::brake(double brakePot) {
  /*
  if(busCurrent < maxBrakeBusCurrent)
  {
    brakePot = 0;
  }
  */

  if (brakePot > 0.3) {brakePot = 0.3;} // LIMIT BRAKING TO 30%

  String ieee754 = IEEE754(brakePot);
  IEEE754ToArray(BRAKE_ARR, ieee754);
  
  Serial.print("BRAKE: ");
  Serial.println(brakePot);
  can_send(0x501, 0, 0, 0, 0, BRAKE_ARR);
  //sendCAN(0x501, BRAKE_ARR);
}

void VehicleController::controlCar(double driveReversePot, double brakePot) {
  // Input potential will be between 0 - 1024
  driveReversePot = driveReversePot / 1023;//max_gas_potential;
  brakePot = brakePot / 1023;//max_brake_potential;

  can_send(0x502, 0, 0, 0, 8, BUS_VOLTAGE);
  //sendCAN(0x502, BUS_VOLTAGE); -- OLD WAY OF SENDING CAN
  /* ---------- NEUTRAL ----------- */
  if (isNeutral) {
    Serial.println("[controlCar()] IS NEUTRAL");
    can_send(0x501, 0, 0, 0, 8, DRIVE_ARR);
    //sendCAN(0x501, DRIVE_ARR); -- OLD WAY OF SENDING CAN
  /* ------------ DRIVING ------------- */
  } else if (isDriving) {
    Serial.println("[controlCar()] IS DRIVING, POT: " + String(driveReversePot));

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);

    //Inserting ieee754 values in DRIVE_ARR
    IEEE754ToArray(DRIVE_ARR, ieee754);

    // Apply brake if driving = 0 & brake > 0
    if(driveReversePot == 0 && brakePot > 0) brake(brakePot);
    else can_send(0x501, 0, 0, 0, 8, DRIVE_ARR); //sendCAN(0x501, DRIVE_ARR); -- OLD WAY OF SENDING CAN
    resetArrays();
    /* --------- REVERSING --------- */
  } else if (isReversing) {
    Serial.println("[controlCar()] IS REVERSING, POT: " + String(driveReversePot));

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);

    //Inserting ieee754 values in DRIVE_ARR
    IEEE754ToArray(REVERSE_ARR, ieee754);

    // Apply brake if reversing = 0 else drive
    if(driveReversePot == 0 && brakePot > 0) brake(brakePot);
    else can_send(0x501, 0, 0, 0, 8, REVERSE_ARR); // sendCAN(0x501, REVERSE_ARR); -- OLD WAY OF SENDING CAN
    resetArrays();
  }
}

/* +++++++++++ CRUISE CONTROL +++++++++++++*/

void VehicleController::enterCruiseControl() {
  if(lastInCruiseControl == 1 && inCruiseControl == 0)
  {
    inCruiseControl = false;
    velocityCruiseControl = 0;
  }
  if(lastInCruiseControl == 0 && inCruiseControl == 1)
  {
    inCruiseControl = true;
    velocityCruiseControl = abs(vehicleVelocity);
  }
}

void VehicleController::applyCruiseControl(int& gas_N_reverse_potential, int& brake_potential) {
  double gas_increment = 3; // CHANGE THIS FOR HARDER ACCELERATION
  double brake_increment = 2;

  // If cruise control is activated and mode is not in reverse
  if(inCruiseControl && drivingMode != 0)
  {
    // ---------------------------------------- TESTTTTTTTTTTTTTTTTTTTTTTTTTTTTT --------------------
    int velocityIncreaseCruise = 2; // For how much to increase cruising speed when pressing increase/decrease
    velocityCruiseControl += (cruiseSpeedIncDec-lastCruiseSpeedIncDec)*velocityIncreaseCruise;
    if(velocityCruiseControl < 0){velocityCruiseControl = 0;} // if velocity 


    // Compute the error between desired and current speed
    float velocityError = velocityCruiseControl - vehicleVelocity; // SetVelocity - CurrentVelocity
    float velocityErrorOffset = 0.1;
    float velocityBrakeErrorOffset = -15;

    if(velocityCruiseControl == 0)
    {
      gas_N_reverse_potential = 0;
      return;
    }

    // If the speed is too low, apply more gas
    if(velocityError > velocityErrorOffset && millis() - lastTimePointVelocityFetched < 1000)
    {
      cruiseBrakeApplied = 0.0;
      brake_potential = 0.0;
      cruiseGasApplied += gas_increment;
      gas_N_reverse_potential = cruiseGasApplied;
    }
    // If the speed is too high, apply more brake
    else if(velocityError < velocityBrakeErrorOffset && millis() - lastTimePointVelocityFetched < 1000)
    {
      cruiseGasApplied = 0.0;
      gas_N_reverse_potential = 0.0;
      cruiseBrakeApplied += brake_increment;
      brake_potential = cruiseBrakeApplied;
    }
    // If the speed is within a tolerance range, maintain the current speed
    else
    {
      float velocityGapError = vehicleVelocity - lastVehicleVelocity;
      cruiseGasApplied = 0.0;
      cruiseBrakeApplied = 0.0;
      brake_potential = 0;

      if(velocityGapError > 0.02)
      {
        potentialCruiseControl -= 1;
        gas_N_reverse_potential = potentialCruiseControl;
      }
      else if(velocityGapError < (-0.02))
      {
        potentialCruiseControl += 1;
        gas_N_reverse_potential = potentialCruiseControl;
      }
      else
      {
        gas_N_reverse_potential = potentialCruiseControl;
      }
      
    
    }

    // Set the gas and brake potentials based on the applied values
    //gas_N_reverse_potential = potentialCruiseControl + cruiseGasApplied;
    //brake_potential = cruiseBrakeApplied;

    /* If potentials are greater or smaller then MIN and MAX
    if(gas_N_reverse_potential > max_gas_potential){gas_N_reverse_potential=max_gas_potential;}
    if(gas_N_reverse_potential < min_gas_potential){gas_N_reverse_potential=min_gas_potential;}
    if(brake_potential > max_brake_potential){brake_potential=max_brake_potential;}
    if(brake_potential < min_brake_potential){brake_potential=min_brake_potential;}*/

    if(gas_N_reverse_potential > Max_gas_N_reverse_potential){gas_N_reverse_potential=Max_gas_N_reverse_potential;}
    if(gas_N_reverse_potential < Min_gas_N_reverse_potential){gas_N_reverse_potential=Min_gas_N_reverse_potential;}
    if(brake_potential > Max_brake_potential){brake_potential=Max_brake_potential;}
    if(brake_potential < Min_brake_potential){brake_potential=Min_brake_potential;}

  }
}

/* ----------- CRUISE CONTROL -----------


/* +++++++++++ ECO CONTROL +++++++++++++*/

void VehicleController::applyECOControl(int& gas_N_reverse_potential) {
  float potential_gap_error = gas_N_reverse_potential - last_gas_N_reverse_potential;

  if(potential_gap_error > 10 && hasAppliedECO)
  {
    int ECOIncrementStep = 1; // Increase this for harder ACCELERATION in ECO

    /*
    debug("GAS BEFORE ECO: " + String(gas_N_reverse_potential));
    */
    ECOPotential += ECOIncrementStep;
    gas_N_reverse_potential = ECOPotential;
    /*
    debug(", GAS AFTER ECO: " + String(gas_N_reverse_potential));
    debugln();
    */
  }
  else if(potential_gap_error > 5 && !hasAppliedECO)
  {
    hasAppliedECO = true;
    ECOPotential = last_gas_N_reverse_potential;
  }
  else
  {
    ECOPotential = 0;
    hasAppliedECO = false;
  }
}

/* ----------- ECO CONTROL ----------- */