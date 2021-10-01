/*
  ThreeAxisArm.cpp - Library for overseeing components of a threeAxis robot arm.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "ThreeAxisArm.h"
 

ThreeAxisArm::ThreeAxisArm(DriveModule& baseJoint, DriveModule& shoulderJoint, DriveModule& elbowJoint, ThreeAxisArmKinematics kinematics, int enablePin) 
: driveModules{ &baseJoint, &shoulderJoint, &elbowJoint }, kinematics(kinematics), enablePin(enablePin)
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

void ThreeAxisArm::setPosition(float x, float y, float z) {
  float destination[3] = {x, y, z};
  float destinationAngles[3];
  kinematics.inverseKinematics(destination, destinationAngles);
  for (int i=0; i<3; i++) {
    driveModules[i]->setPosition(destinationAngles[i], ABSOLUTE);
    Serial.println("currentAngle on motor " + (String) i + ": " + (String) driveModules[i]->getCurrentAngle());
  }

  Serial.print("Angles: ");
  for (float angle: destinationAngles) Serial.print((String) angle + " ");
  Serial.println("");
}

void ThreeAxisArm::setJointAngle(float angle, joints joint) {
  driveModules[joint]->setPosition(angle, ABSOLUTE);
}

boolean ThreeAxisArm::isAtRest() {
  for (DriveModule* module : driveModules) if (!(*module).isAtRest()) return false;
  return true;
}
float * ThreeAxisArm::getPosition() {
  
}

//            private methods


// void moveMotor(joints joint, float distance, positionMode posMode);
//     void moveMotor(float distance[3], positionMode posMode);