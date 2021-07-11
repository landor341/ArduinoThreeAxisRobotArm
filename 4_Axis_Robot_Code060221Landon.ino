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

const int dirPins[numJoints] = {5, 6, 7};
const int stepPins[numJoints] = {2, 3, 4};
const int driveEnablePin=8;

const int motorStepsPerRev=200; //steps on the actual stepper motor
const int microStepRatio = 4; //4 microsteps / 1 full step
const float gearRatios[3] = {5, 4.25, 4.25}; //5 motor rotations / 1 joint


const float stepsPerRev[3] = {
  motorStepsPerRev * microStepRatio * gearRatios[0],
  motorStepsPerRev * microStepRatio * gearRatios[1],
  motorStepsPerRev * microStepRatio * gearRatios[2]
};
const float degreesPerStep[3] = {360. / stepsPerRev[0], 360. / stepsPerRev[1], 360. / stepsPerRev[2]};

int jointSteps[numJoints] = {0}; //initialize all values in array to 0
int jointSetSteps[numJoints] = {0};

double currentMicros = 0;

ThreeAxisArmKinematics kinematics(130, 160, 142.9, 25.4); 

a4988DriveModule joint0Motor(driveEnablePin, dirPins[0], stepPins[0], stepsPerRev[0], 1, 2, 350); 
a4988DriveModule joint1Motor(driveEnablePin, dirPins[1], stepPins[1], stepsPerRev[1], 1, 2, 350); 
a4988DriveModule joint2Motor(driveEnablePin, dirPins[2], stepPins[2], stepsPerRev[2], 1, 2, 350); 

ThreeAxisArm arm();

void setup() {
  joint0Motor.enableMotor(true);
  joint1Motor.enableMotor(true);
  joint2Motor.enableMotor(true);
}

void loop() {
  currentMicros = micros();
  joint0Motor.setPosition(round(joint0Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);
  joint1Motor.setPosition(round(joint1Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);
  joint2Motor.setPosition(round(joint2Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);

  double initialTime = currentMicros;
  int i = 0;
  while ((!joint0Motor.atRest() && !joint1Motor.atRest() && !joint2Motor.atRest())) {
    currentMicros = micros();
    joint0Motor.update(currentMicros);
    joint1Motor.update(currentMicros);
    joint2Motor.update(currentMicros);
    i++;
  }
  currentMicros = micros();
  Serial.println((String) joint0Motor.currentSteps + "  " + (String) joint0Motor.setSteps);
  Serial.println("cps of: " + (String) i + " per " + (String) ((currentMicros - initialTime)/1000000.) + " seconds");//(i / ((currentMicros - initialTime)/1000000.)));
}
