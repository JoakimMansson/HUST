#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>


class VehicleController
{

    public:
        unsigned char DRIVE_ARR[8] = { 0, 0, 250, 68, 0, 0, 0, 0 }; // 2000 RPM
        unsigned char REVERSE_ARR[8] = { 0, 0, 250, 196, 0, 0, 0, 0 }; // -2000 RPM
        unsigned char NEUTRAL_ARR[8] = { 0, 0, 250, 68, 0, 0, 0, 0 }; // 2000 RPM
        unsigned char BRAKE_ARR[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        unsigned char BUS_VOLTAGE[8] = { 0, 0, 0, 0, 0, 0, 128, 63};
    
        int drivingMode = 0; /* 0 = Neutral, 1 = Drive, 2 = Reverse */
        bool inECO = true;
        bool inCruiseControl = false;
        bool inStartScreen = true;

    /* 
            *IMPORTANT* 
        UPDATE THESE MIN/MAX POTENTIALS 
    */
        int Max_gas_N_reverse_potential = 0;
        int Min_gas_N_reverse_potential = 0;
        int Max_brake_potential = 0;
        int Min_brake_potential = 0;

        SoftwareSerial *can_serial;

        void vehicleControlLoop(); // Main function to drive vehicle
        VehicleController(SoftwareSerial can_serial); // Constructor
        

    private:
        bool isNeutral = true;
        bool isDriving = false;
        bool isReversing = false;
        bool isBraking = false;

        float last_brake_potential = 1/1337; 
        float last_gas_N_reverse_potential = 1/1337;

        // Input for driving modes ECO/RACING
        bool lastInECO = true;
        bool hasAppliedECO = false;
        int ECOPotential = 0;


        // Input for cruise control and buttons to decrease and increase cruise speed
        
        bool lastInCruiseControl = false;
        float potentialCruiseControl = 0.0; // Regulated when in limits of set cruise velocity
        float velocityCruiseControl = 0.0; // Set at the initiation of cruise
        float cruiseBrakeApplied = 0.0; // Used for accelerating brake if speed does not decrease
        float cruiseGasApplied = 0.0; // Used for accelerating gas if speed does not increase
        int cruiseSpeedIncDec = 100;
        int lastCruiseSpeedIncDec = 100;

        double vehicleVelocity = 0.0; // Velocity of vehicle
        double lastVehicleVelocity = 0.0; // Previous iteration of vehicle velocity

        void IEEE754ToArray(unsigned char (&brake_drive_reverse)[8], String ieee754);
        String IEEE754(const double potential);
        double hexStringToInt(const char *hexString);
        void resetArrays();

        void updateCurrentDrivingMode();

        void brake(double brakePot);
        void controlCar(double driveReversePot, double brakePot);

        void enterCruiseControl();
        void applyCruiseControl(float& gas_N_reverse_potential, float& brake_potential);

        void applyECOControl(float& gas_N_reverse_potential);

        

};