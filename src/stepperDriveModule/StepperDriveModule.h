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
  int accelerationLimit, ticksPerRevolution, maxVelocity;
  stepperMotor(int enablePin, int dirPin, int stepPin, int ticksPerRevolution, int maxVelocity, int accelerationLimit)
  : enablePin(enablePin), dirPin(dirPin), stepPin(stepPin), 
    ticksPerRevolution(ticksPerRevolution), maxVelocity(maxVelocity), accelerationLimit(accelerationLimit) 
    { }
};

class StepperDriveModule : public DriveModule
{
  public:
    virtual int getCurrentSteps()=0;
    virtual float getCurrentAngle()=0;
    virtual float getCurrentVelocity()=0;
    
    virtual void enableMotor(boolean enable)=0;
    virtual void setAngleOffset(float angleOffset)=0;

    virtual boolean isAtRest()=0;

    virtual boolean setPosition(float distance, positionMode posMode)=0;
    virtual boolean setPosition(float distance, positionMode posMode, motorUnits unit)=0;

    virtual void update(double microsTime)=0;
    virtual void halt()=0;
    virtual void zero()=0;
    int currentSteps = 0;
    int desiredPosition = 0;


  private:
    virtual void setDir(boolean clockwise)=0;
    virtual boolean incrementMotor()=0;
    virtual int degreesToTicks(float angle)=0;
    virtual float ticksToDegrees(int ticks)=0;
    virtual int rotationsToTicks(float rotations)=0;
    virtual int ticksToRotations(int ticks)=0;

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