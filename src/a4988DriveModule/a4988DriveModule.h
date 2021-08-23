/*
  a4988DriveModule.cpp - Library for simplifying interaction with a4988DriveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef a4988DriveModule_h
#define a4988DriveModule_h

#include "Arduino.h"
#include "../StepperDriveModule/StepperDriveModule.h"
#include "../DriveModule/DriveModule.h"

class a4988DriveModule : public DriveModule
{
  public:

    a4988DriveModule(stepperMotor motor, int gearRatio, int microStepsPerStep, float maxAngle, int limitPin, direction dirToSwitch);
    
    int getCurrentSteps();
    float getCurrentAngle();
    float getCurrentVelocity();

    void enableMotor(boolean enable);
    void setAngleOffset(float angleOffset);

    boolean isAtRest();
    
    // boolean setPosition(float distance, positionMode posMode);
    boolean setPosition(float distance, positionMode posMode);
    boolean setPosition(float distance, positionMode posMode, motorUnits units);

    void update(double microsTime);
    void halt();
    void zero();
    int currentPosition = 0;
    int desiredPosition = 0;


  private:
    void setDir(boolean clockwise);
    boolean incrementMotor();
    int degreesToTicks(float angle);
    float ticksToDegrees(int ticks);
    int rotationsToTicks(float rotations);
    int ticksToRotations(int ticks);
    bool atSwitch();

    int ticksPerRevolution;
    
    unsigned int enablePin;
    unsigned int dirPin;
    unsigned int stepPin;
    unsigned int limitPin;
    direction dirToSwitch;
    unsigned int maxVelocity;
    unsigned int acceleration;
    int maxTicks;
    unsigned int maxAngle;


    double lastIncrementTime = 0;
    int currentVelocity = 0;

    int angleOffset = 0;
    boolean motorEnabled {false};
    boolean motorClockwise {true};
};

#endif