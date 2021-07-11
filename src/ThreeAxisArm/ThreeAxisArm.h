/*
  a4988DriveModule.cpp - Library for simplifying interaction with a4988DriveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef ThreeAxisArm_h
#define ThreeAxisArm_h

#include "Arduino.h"
#include "../DriveModule/DriveModule.h"

enum joints {
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2
};

class ThreeAxisArm
{
  public:


    ThreeAxisArm(DriveModule& baseJoint, DriveModule& shoulderJoint, DriveModule& elbowJoint);

    void calibrateMotors(int enablePin[3], int dirPin[3], int stepPin[3], int limitPin[3], int motorTicksPerRevolution[3], int maxRPM[3], float rpmAcceleration[3], float maxAngle[3], direction dirToSwitch[3]); //also zeroes

    void zeroMotors(int limitSwitchPins[], direction dirToSwitch[3]);

    void setPosition(float x, float y, float z);

    float * getPosition();

    void setJointAngle(float angle, int joint);

    void update();

    void halt();

    boolean atRest();
    
  private:
    void moveMotor(joints joint, float distance, positionMode posMode);
    void moveMotor(float distance[3], positionMode posMode);

    DriveModule * driveModules[3];
};

#endif