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
    ThreeAxisArmKinematics(float joint2Length, float joint3Length, float j1Offs, float j2Offs);
    float * forwardKinematics(float angles[3]);
    float * inverseKinematics(float pos[3]);
    float * getPosition();
    float * getAngles();
  private:
    float joint2Length_;
    float joint3Length_;
    float j1Offs_;
    float j2Offs_;
    float coords[3];
    float angles[3];
    
    const float Pi = 3.1415926; //not overkill dw, float cuts it off too
};

#endif
