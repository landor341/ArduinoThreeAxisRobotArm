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

struct stepperMotor {
  int enablePin;
  int dirPin;
  int stepPin;
  int ticksPerRevolution;
  int maxTPS;
  int maxTickAccel;
  stepperMotor(int enablePin, int dirPin, int stepPin, int ticksPerRevolution, int maxTPS, int maxTickAccel)
  : enablePin(enablePin),
    dirPin(dirPin),
    stepPin(stepPin),
    ticksPerRevolution(ticksPerRevolution),
    maxTPS(maxTPS),
    maxTickAccel(maxTickAccel)
  { }
};

class a4988DriveModule
{
  public:

    a4988DriveModule(stepperMotor motor, int gearRatio, int microStepsPerStep, float maxAngle, int limitPin, direction dirToSwitch);
    
    int getCurrentSteps();
    float getCurrentAngle();
    float getCurrentVelocity();
    
    void enableMotor(boolean enable);
    void setAngleOffset(float angleOffset);

    boolean atRest();

    boolean setPosition(float distance, positionMode posMode, motorUnits units);

    void update(double microsTime);
    void halt();
    void zero();
    int currentSteps = 0;
    int setSteps = 0;


  private:
    void setDir(boolean clockwise);
    boolean incrementMotor();
    int degreesToTicks(float angle);
    float ticksToDegrees(int ticks);
    int rotationsToTicks(float rotations);

    unsigned int enablePin;
    unsigned int dirPin;
    unsigned int stepPin;
    unsigned int limitPin;
    direction dirToSwitch;
    unsigned int maxTPS;
    unsigned int acceleration;
    unsigned int maxTicks;

    int ticksPerRevolution;

    double lastIncrementTime = 0;
    int currentVelocity = 0;

    int angleOffset = 0;
    boolean motorEnabled = false;
    boolean motorClockwise = true;
};

#endif