/*
  ThreeAxisArmKinematics.cpp - Library for calculating 3 axis robot arm kinematics.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef ThreeAxisArmKinematics_h
#define ThreeAxisArmKinematics_h

#include "Arduino.h"

class ThreeAxisArmKinematics
{
  public:
    ThreeAxisArmKinematics(float shoulderJointLength, float elbowJointLength, float baseJointOffset, float shoulderJointOffset);
    float * forwardKinematics(float angles[3]);
    float * inverseKinematics(float pos[3]);
    float * getPosition();
    float * getAngles();
  private:
    float shoulderJointLength;
    float elbowJointLength;
    float baseJointOffset;
    float shoulderJointOffset;
    float coords[3];
    float angles[3];
    
    const float Pi = 3.1415926; //not overkill dw, float cuts it off too
};

#endif
