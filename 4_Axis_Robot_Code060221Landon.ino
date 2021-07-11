/*
 * motor controller reference: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
 * arduino cam possibility: https://circuitdigest.com/microcontroller-projects/how-to-use-ov7670-camera-module-with-arduino
 * 
 *                                        Questions or ideas
 * make another inverse kinematics that allows you to pass a vector  
 * I think the kinematics assumes the second joint is at z=0, negative z's might work but it's confusing
 *  Deal with time function overflows
 */

#include "src/ThreeAxisArmKinematics/ThreeAxisArmKinematics.h"
#include "src/a4988DriveModule/a4988DriveModule.h"
#include "src/ThreeAxisArm/ThreeAxisArm.h"

const int numJoints = 3;

const int motorStepsPerRev=200; //steps on the actual stepper motor
const int microStepRatio = 4; //4 microsteps / 1 full step
const float gearRatios[numJoints] = {5, 4.25, 4.25}; //5 motor rotations / 1 joint

const float degreesPerStep[numJoints] = {360. / stepsPerRev[BASE], 360. / stepsPerRev[SHOULDER], 360. / stepsPerRev[ELBOW]};

const int dirPins[numJoints] = {5, 6, 7};
const int stepPins[numJoints] = {2, 3, 4};
const int driveEnablePin=8;

ThreeAxisArmKinematics kinematics(130, 160, 142.9, 25.4); 

stepperMotor motors[numJoints] = {
  stepperMotor(driveEnablePin, dirPins[BASE], stepPins[BASE], 200, 100, 200),
  stepperMotor(driveEnablePin, dirPins[SHOULDER], stepPins[SHOULDER], 200, 100, 200),
  stepperMotor(driveEnablePin, dirPins[ELBOW], stepPins[ELBOW], 200, 100, 200)
};

a4988DriveModule baseJoint(motors[BASE], gearRatios[BASE], microStepRatio, 0, 0, CLOCKWISE); 
a4988DriveModule shoulderJoint(motors[BASE], gearRatios[BASE], microStepRatio, 0, 0, CLOCKWISE); 
a4988DriveModule elbowJoint(motors[BASE], gearRatios[BASE], microStepRatio, 0, 0, CLOCKWISE); 

ThreeAxisArm arm(baseJoint, shoulderJoint, elbowJoint);

const float stepsPerRev[numJoints] = {
  motorStepsPerRev * microStepRatio * gearRatios[BASE],
  motorStepsPerRev * microStepRatio * gearRatios[SHOULDER],
  motorStepsPerRev * microStepRatio * gearRatios[ELBOW]
};

int jointSteps[numJoints] = {0}; //initialize all values in array to 0
int jointSetSteps[numJoints] = {0};
double currentMicros = 0;

void setup() {
  baseJoint.enableMotor(true);
  shoulderJoint.enableMotor(true);
  elbowJoint.enableMotor(true);
}

void loop() {
  currentMicros = micros();
  baseJoint.setPosition(round(baseJoint.getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);
  shoulderJoint.setPosition(round(shoulderJoint.getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);
  elbowJoint.setPosition(round(elbowJoint.getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);

  double initialTime = currentMicros;
  int i = 0;
  while ((!baseJoint.isAtRest() && !shoulderJoint.isAtRest() && !elbowJoint.isAtRest())) {
    currentMicros = micros();
    baseJoint.update(currentMicros);
    shoulderJoint.update(currentMicros);
    elbowJoint.update(currentMicros);
    i++;
  }
  currentMicros = micros();
  Serial.println((String) baseJoint.currentPosition + "  " + (String) baseJoint.desiredPosition);
  Serial.println("cps of: " + (String) i + " per " + (String) ((currentMicros - initialTime)/1000000.) + " seconds");//(i / ((currentMicros - initialTime)/1000000.)));
}
