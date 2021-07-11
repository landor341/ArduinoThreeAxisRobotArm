/*
  StepperDriveModule.h - generic interface for StepperDriveModules to implement.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef StepperDriveModule_h
#define StepperDriveModule_h

#include "Arduino.h"
#include "../driveModule/DriveModule.h"

enum motorUnits {
  TICKS = 1,
  ANGLE = 0
};

struct stepperMotor {
  int enablePin, dirPin, stepPin;
  int maxTickAccel, ticksPerRevolution, maxTPS;
  stepperMotor(int enablePin, int dirPin, int stepPin, int ticksPerRevolution, int maxTPS, int maxTickAccel)
  : enablePin(enablePin), dirPin(dirPin), stepPin(stepPin), 
    ticksPerRevolution(ticksPerRevolution), maxTPS(maxTPS), maxTickAccel(maxTickAccel) 
    { }
};

class StepperDriveModule : public DriveModule
{
  public:

    StepperDriveModule(stepperMotor motor, int gearRatio, int microStepsPerStep, float maxAngle, int limitPin, direction dirToLimitSwitch);
    
    int getCurrentSteps();
    float getCurrentAngle();
    float getCurrentVelocity();
    
    void enableMotor(boolean enable);
    void setAngleOffset(float angleOffset);

    boolean atRest();

    boolean setPosition(float distance, positionMode posMode);
    boolean setPosition(float distance, positionMode posMode, motorUnits unit);

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