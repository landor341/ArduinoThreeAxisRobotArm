/*
  ThreeAxisArm.cpp - Library for overseeing components of a threeAxis robot arm.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "ThreeAxisArm.h"

void ThreeAxisArm::calibrateMotors(int enablePin[3], int dirPin[3], int stepPin[3], int motorTicksPerRevolution[3], int maxRPM[3], int RPMAcceleration[3], float maxAngle[3]) { //also zeroes
    driveModules[0] = a4988DriveModule(enablePin[0], dirPin[0], stepPin[0], motorTicksPerRevolution[0], maxRPM[0], RPMAcceleration[0], maxAngle[0]);
    driveModules[1] = a4988DriveModule(enablePin[1], dirPin[1], stepPin[1], motorTicksPerRevolution[1], maxRPM[1], RPMAcceleration[1], maxAngle[1]);
    driveModules[2] = a4988DriveModule(enablePin[2], dirPin[2], stepPin[2], motorTicksPerRevolution[2], maxRPM[2], RPMAcceleration[2], maxAngle[2]);
}

void ThreeAxisArm::update() {
    double time = micros();
    for (a4988DriveModule module : driveModules) module.update(time);
}

void ThreeAxisArm::halt() {

}

boolean ThreeAxisArm::atRest() {

}
    
// a4988DriveModule drivemodules[3];