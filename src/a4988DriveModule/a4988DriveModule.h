/*
  a4988DriveModule.cpp - Library for simplifying interaction with a4988DriveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef a4988DriveModule_h
#define a4988DriveModule_h

#include "Arduino.h"

enum motorUnits {
  TICKS = 1,
  ANGLE = 0
};
enum positionMode {
  ABSOLUTE = 1,
  RELATIVE = 0
};
enum direction {
  COUNTERCLOCKWISE = 0,
  CLOCKWISE = 1
};

class a4988DriveModule
{
  public:

    a4988DriveModule(int enablePin, int dirPin, int stepPin, int limitPin, int motorTicksPerRevolution, int maxRPM, float rpmAcceleration, float maxAngle, direction dirToSwitch);
    
    int getCurrentSteps();
    float getCurrentAngle();
    float getCurrentVelocity();
    
    void enableMotor(boolean enable);
    void setDir(boolean clockwise);
    void setAngleOffset(float angleOffset);

    boolean atRest();

    boolean setPosition(float distance, positionMode posMode, motorUnits units);

    void update(double microsTime);
    void halt();
    void zero();
    int currentSteps = 0;
    int setSteps = 0;


  private:
    boolean incrementMotor();
    int degreesToTicks(float angle);
    float ticksToDegrees(int ticks);
    int rotationsToTicks(float rotations);

    unsigned int enablePin_;
    unsigned int dirPin_;
    unsigned int stepPin_;
    unsigned int limitPin_;
    direction dirToSwitch_;
    unsigned int maxTPS_;
    unsigned int TPSAcceleration_;
    unsigned int maxTicks_;

    int motorTicksPerRevolution_;

    double lastIncrementTime = 0;
    int currentVelocity = 0;

    int angleOffset_ = 0;
    boolean motorEnabled = false;
    boolean motorClockwise = true;
};

#endif