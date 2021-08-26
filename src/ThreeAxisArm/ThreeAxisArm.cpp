/*
  ThreeAxisArm.cpp - Library for overseeing components of a threeAxis robot arm.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "ThreeAxisArm.h"
 

ThreeAxisArm::ThreeAxisArm(DriveModule& baseJoint, DriveModule& shoulderJoint, DriveModule& elbowJoint, int enablePin) 
: driveModules{ &baseJoint, &shoulderJoint, &elbowJoint }, enablePin(enablePin)
  {}

void ThreeAxisArm::enableArm(boolean en) {
  digitalWrite(enablePin, !en);
  if (en) zero();
}

void ThreeAxisArm::update() {
  double time = micros();
  for (DriveModule* module : driveModules) module->update(time);
}
void ThreeAxisArm::halt() { 
  for (DriveModule* module : driveModules) module->halt(); 
  enableArm(false);
}

void ThreeAxisArm::zero() { 
  for (int i=2; i>=0; i--) driveModules[i]->enableMotor(true);
  for (int i=2; i>=0; i--) driveModules[i]->zero();
}

// void setPosition(float x, float y, float z);
//     void setJointAngle(float angle, joints joint);
    // boolean isAtRest();
    // float * getPosition();

//            private methods


// void moveMotor(joints joint, float distance, positionMode posMode);
//     void moveMotor(float distance[3], positionMode posMode);