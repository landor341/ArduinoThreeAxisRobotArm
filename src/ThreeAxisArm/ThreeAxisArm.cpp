/*
  ThreeAxisArm.cpp - Library for overseeing components of a threeAxis robot arm.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "ThreeAxisArm.h"
 

ThreeAxisArm::ThreeAxisArm(DriveModule& baseJoint, DriveModule& shoulderJoint, DriveModule& elbowJoint) 
: driveModules{ &baseJoint, &shoulderJoint, &elbowJoint }
  {}

void ThreeAxisArm::update() {
    double time = micros();
    for (DriveModule* module : driveModules) module->update(time);
}
void ThreeAxisArm::halt() { for (DriveModule* module : driveModules) module->halt(); }
void ThreeAxisArm::zero() { for (DriveModule* module : driveModules) module->zero(); }