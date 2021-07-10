/*
  a4988DriveModule.cpp - Library for simplifying interaction with a4988DriveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef ThreeAxisArm_h
#define ThreeAxisArm_h

#include "Arduino.h"
#include "../a4988DriveModule/a4988DriveModule.h"


enum motorUnits {
  TICKS = 0,
  ANGLE = 1
};
enum positionMode {
  ABSOLUTE = 0,
  RELATIVE = 1
};
enum joints {
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2
};

class ThreeAxisArm
{
  public:


    ThreeAxisArm();

    void calibrateMotors(int enablePin[3], int dirPin[3], int stepPin[3], int motorTicksPerRevolution[3], int maxRPM[3], int RPMAcceleration[3], float maxAngle[3]); //also zeroes

    void zeroMotors(int limitSwitchPins[], boolean clockwiseToSwitch[]);

    void setPosition(float x, float y, float z);

    float * getPosition();

    void setJointAngle(float angle, int joint);

    void update();

    void halt();

    boolean atRest();
    
  private:
    void moveMotor(int motorNum, float distance, positionMode posMode, motorUnits unit);
    void moveMotor(int motorNum, float numDegrees);
    void moveMotor(float numDegrees[3]);

    a4988DriveModule driveModules[3];
};

#endif