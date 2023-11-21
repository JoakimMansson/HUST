#include "VehicleController.h"
#include "Serial_CAN_FD.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "CANDecoder.h"
#include "GlobalVariablesReceive.h"


void VehicleController::vehicleControlLoop(int gas_reverse_pot, int brake_pot) {
  int gas_N_reverse_potential = gas_reverse_pot;
  int brake_potential = brake_pot;
  //Serial.println(gas_N_reverse_potential);
  updateCurrentDrivingMode();
  //enterCruiseControl();
  
  /*
  debugln("gas_N_reverse_potential before: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  */

  // EXIT CRUISE IF POTENTIAL > 100 is applied
  if (inCruiseControl && lastVehicleVelocity != 1/1337 && gas_reverse_pot - last_gas_N_reverse_potential > 30) {
    inCruiseControl = false;
  }
  applyCruiseControl(gas_N_reverse_potential, brake_potential); // IF IN CRUISE CONTROL UPDATE gas_N_reverse_potential and brake_potential

  // Sends drive commands to vehicle
  controlCar((float)gas_N_reverse_potential, (float)brake_potential);

  last_gas_N_reverse_potential = gas_N_reverse_potential;
  last_brake_potential = brake_potential;
  //lastVehicleVelocity = vehicleVelocity; // Set last vehicle velocity to current velocity
  /*
  debugln("gas_N_reverse_potential after: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  debugln("Vehicle velocity: " + String(vehicleVelocity) + ", velocityCruiseControl: " + String(velocityCruiseControl));
  */
  lastCruiseSpeedIncDec = cruiseSpeedIncDec;
  lastInCruiseControl = inCruiseControl;
}

void VehicleController::decreaseCruiseSpeed() {
  velocityCruiseControl -= 2;
  if (velocityCruiseControl < 0) velocityCruiseControl = 0;
}

void VehicleController::IncreaseCruiseSpeed() {
  velocityCruiseControl += 2;
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

  String ieee754 = IEEE754(brakePot);
  IEEE754ToArray(BRAKE_ARR, ieee754);
  
  //Serial.print("BRAKE: ");
  //Serial.println(brakePot);

  String serialData = "501";
  for (int i = 0; i < 8; i++) {
    serialData += " " + String(BRAKE_ARR[i]);
  }
  Serial.println(serialData);
  
}

void VehicleController::controlCar(float driveReversePot, float brakePot) {
  /* OFFSET FOR POTENTIOMETERS */
  driveReversePot = driveReversePot - 5;
  brakePot = brakePot - 5;

  /* CHECK IF NEW VALUE IS < SET POTENTIALS IN START_SCREEN */
  if (driveReversePot < Min_gas_N_reverse_potential) driveReversePot = Min_gas_N_reverse_potential;
  if (brakePot < Min_brake_potential) brakePot = Min_brake_potential;

  driveReversePot = mapFloat(driveReversePot, (float)Min_gas_N_reverse_potential, (float)Max_gas_N_reverse_potential, 0, 1) - 0.10;//max_gas_potential;
  brakePot = mapFloat(brakePot, (float)Min_brake_potential, (float)Max_brake_potential, 0, 0.1870);//max_brake_potential;

  driveReversePot = ((int)(driveReversePot*100))/100.0;
  brakePot = ((int)(brakePot*100))/100.0;

  if (0.3 < driveReversePot) driveReversePot += 0.02;
  if (0.4 < driveReversePot) driveReversePot += 0.02;
  if (0.5 < driveReversePot) driveReversePot += 0.02;
  if (0.6 < driveReversePot) driveReversePot += 0.02;
  if (0.7 < driveReversePot) driveReversePot += 0.02;

  if (driveReversePot < 0) driveReversePot = 0;
  if (driveReversePot > 1) driveReversePot = 1;
  if (brakePot < 0) brakePot = 0;
  if (brakePot > 1) brakePot = 1;


  if (brakePot > 0.06) inCruiseControl = false; // Exit Cruise Control if electronic braking


  String serialData = "502";
  for (int i = 0; i < 8; i++) {
    serialData += " " + String(BUS_VOLTAGE[i]);
  }
  Serial.println(serialData);

  /* ---------- NEUTRAL ----------- */
  if (isNeutral) {

  String serialData = "501";
  for (int i = 0; i < 8; i++) {
    serialData += " " + String(NEUTRAL_ARR[i]);
  }
  Serial.println(serialData);

  /* ------------ DRIVING ------------- */
  } else if (isDriving) {

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);
    //IEEE754ToArray(DRIVE_ARR, ieee754);
    
        
    if(driveReversePot == 0 && brakePot > 0 && abs(vehicleVelocity) >= 1) brake(brakePot);
    else {

      //Inserting ieee754 values in DRIVE_ARR
      IEEE754ToArray(DRIVE_ARR, ieee754);
      String serialData = "501";
      for (int i = 0; i < 8; i++) {
        serialData += " " + String(DRIVE_ARR[i]);
      }
      Serial.println(serialData);
    }

    /* --------- REVERSING --------- */
  } else if (isReversing) {

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);
    
    if(driveReversePot == 0 && brakePot > 0 && abs(vehicleVelocity) >= 1) brake(brakePot);
    else {

      IEEE754ToArray(REVERSE_ARR, ieee754);
      String serialData = "501";
      for (int i = 0; i < 8; i++) {
        serialData += " " + String(REVERSE_ARR[i]);
      }
      Serial.println(serialData);
    }
    
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
  const int gas_increment = 1; // CHANGE THIS FOR HARDER ACCELERATION
  const int brake_increment = 2;
  //static int cruiseBrakeApplied = 0; // Used for accelerating brake if speed does not decrease
  static int cruiseGasApplied = Min_gas_N_reverse_potential; // Used for accelerating gas if speed does not increase

  // If cruise control is activated and mode is not in reverse
  if(inCruiseControl && drivingMode == 1)
  {
    // Compute the error between desired and current speed
    int velocityError = velocityCruiseControl - vehicleVelocity; // SetVelocity - CurrentVelocity
    const int velocityErrorOffset = 0.1;
    const int velocityTopMaxErrorOffset = -1;

    if(velocityCruiseControl == 0)
    {
      gas_N_reverse_potential = Min_gas_N_reverse_potential;
      return;
    }

    // If the speed is too low, apply more gas
    if(velocityError >= velocityErrorOffset)
    {
      // If car is accelerating dont increase the gas potential more  
      if (vehicleVelocity - lastVehicleVelocity < 0.1) {
        cruiseGasApplied += gas_increment;
        gas_N_reverse_potential = cruiseGasApplied;
      }
      
    }
    
    // -- BRAKE DISABLED IN CRUISE CONTROL --
    // If the speed is too high, apply more brake
    else if(velocityError < velocityTopMaxErrorOffset)
    {
      cruiseGasApplied = Min_gas_N_reverse_potential;
      gas_N_reverse_potential = Min_gas_N_reverse_potential;
    }
    /*
    // If the speed is within a tolerance range, maintain the current speed
    else
    {
      int cruiseVelocityGapError = vehicleVelocity - velocityCruiseControl;
      int lastVelocityGapError = vehicleVelocity - lastVehicleVelocity;
      //cruiseGasApplied = Min_gas_N_reverse_potential;
      //brake_potential = 0;
      
      if (cruiseVelocityGapError >= 2)
      {
        if (lastVelocityGapError >= 0) {
          cruiseGasApplied -= 1;
          gas_N_reverse_potential = cruiseGasApplied;
        }
        
      }
      else if (lastVelocityGapError < 0 && velocityCruiseControl < vehicleVelocity && vehicleVelocity < velocityCruiseControl + 2)
      {
        cruiseGasApplied += 1;
        gas_N_reverse_potential = cruiseGasApplied;
      }
      else
      {
        gas_N_reverse_potential = cruiseGasApplied;
      }
    }
    */

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

  if (inECO) {

  }
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

float VehicleController::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}