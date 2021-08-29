/*
  a4988DriveModule.cpp - Library for simplifying interaction with a4988DriveModules.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef ThreeAxisArm_h
#define ThreeAxisArm_h

#include "Arduino.h"
#include "../DriveModule/DriveModule.h"
#include "../ThreeAxisArmKinematics/ThreeAxisArmKinematics.h"

enum joints {
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2
};

class ThreeAxisArm
{
  public:
    ThreeAxisArm(DriveModule& baseJoint, DriveModule& shoulderJoint, DriveModule& elbowJoint, ThreeAxisArmKinematics kinematics, int enablePin);

    void update();
    void halt();

    void zero();
    void setPosition(float x, float y, float z);
    void setJointAngle(float angle, joints joint);
    void enableArm(boolean en);

    int enablePin;
    boolean isAtRest();
    float * getPosition();
    
  private:
    void moveMotor(joints joint, float distance, positionMode posMode);
    void moveMotor(float distance[3], positionMode posMode);

    DriveModule * driveModules[3];
    ThreeAxisArmKinematics * kinematics;
    float position[3] = { 0 };
};

#endif