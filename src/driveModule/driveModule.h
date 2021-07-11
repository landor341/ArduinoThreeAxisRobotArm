/*
  driveModule.h - generic interface for driveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef DriveModule_h
#define DriveModule_h

#include "Arduino.h"


enum positionMode {
  ABSOLUTE = 1,
  RELATIVE = 0
};
enum direction {
  COUNTERCLOCKWISE = 0,
  CLOCKWISE = 1
};

class DriveModule
{
  public:
    virtual float getCurrentAngle()=0;
    virtual float getCurrentVelocity()=0;
    
    virtual void setAngleOffset(float angleOffset)=0;

    virtual boolean isAtRest()=0;

    virtual boolean setPosition(float distance, positionMode posMode)=0;

    virtual void update(double microsTime)=0;
    virtual void halt()=0;
    virtual void zero()=0;
    int currentPosition = 0;
    int desiredPosition = 0;


  private:
    unsigned int maxVelocity;
    unsigned int acceleration;
    unsigned int maxAngle;
    int currentVelocity { 0 };
    int angleOffset { 0 };
    boolean motorEnabled { false };
};

#endif