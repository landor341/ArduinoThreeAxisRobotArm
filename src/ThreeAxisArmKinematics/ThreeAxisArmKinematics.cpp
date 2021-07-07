/*
  ThreeAxisArmKinematics.cpp - Library for calculating 3 axis robot arm kinematics.
  Created by Landon R. Faris, June 2, 2021.
*/

#include "Arduino.h"
#include "ThreeAxisArmKinematics.h"



ThreeAxisArmKinematics::ThreeAxisArmKinematics(float joint2Length, float joint3Length, float j1Offs, float j2Offs) {
    joint2Length_ = joint2Length;
    joint3Length_ = joint3Length;
    j1Offs_ = j1Offs;
    j2Offs_ = j2Offs;
}
float * ThreeAxisArmKinematics::forwardKinematics(float angles[3]) {
    float currentRadians[3];
    currentRadians[0] = angles[0] * Pi/180;
    currentRadians[1] = (angles[1] * Pi/180);
    currentRadians[2] = (angles[2] + (180 - angles[1])) * Pi/180;

    float radius = (cos(currentRadians[1]) * joint2Length_) + (cos(currentRadians[2]) * joint3Length_) + j2Offs_;
    coords[0] = cos(currentRadians[0]) * coords[3];
    coords[1] = sin(currentRadians[0]) * coords[3];
    coords[2] = ((sin(currentRadians[1]) * joint2Length_) + (sin(currentRadians[2]) * joint3Length_)) + j1Offs_;
    
    inverseKinematics(coords);
    return coords;
}
float * ThreeAxisArmKinematics::inverseKinematics(float pos[3]) {
  float x = pos[0];
  float y = pos[1];
  float z = pos[2];

  float sqJ2Length=sq(joint2Length_), sqJ3Length=sq(joint3Length_);
  
  // xy vector
  angles[0] = atan(y/x); //angle
  float radius = sqrt(sq(x)+sq(y)); //magnitude
  
  // (xy)z vector
  float theta = atan(z/radius);
  float c = sqrt(sq(radius)+sq(z)); 

  // Joint 2 and 3 angles based on xyz vector
  angles[1] = acos((sqJ2Length + sq(c) - sqJ3Length) / (2 * joint2Length_ * c)); 
  angles[2] = acos((sqJ2Length + sqJ3Length - sq(c)) / (2 * joint2Length_ * joint3Length_)); 
  
  // radian to degree conversions
  angles[0] = angles[0]/Pi*180; 
  if (x<0) angles[0] = (angles[0]-180);
  
  // adjust for xyz vector and convert to degree
  angles[1] = ((angles[1] + theta) / Pi*180);
  angles[2] = (((angles[2]) / Pi*180) - (180 - angles[1]));

  forwardKinematics(angles);
  return angles;
}
float * ThreeAxisArmKinematics::getPosition() {
  return coords;
}
float * ThreeAxisArmKinematics::getAngles() {
  return angles;
}
