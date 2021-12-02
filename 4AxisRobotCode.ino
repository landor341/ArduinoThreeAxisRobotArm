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


const unsigned int dirPins[numJoints] = {5, 6, 7};
const unsigned int stepPins[numJoints] = {2, 3, 4};
const unsigned int limitPins[numJoints] = {9, 10, 11};
const int driveEnablePin = 8;

// int maxTPS[numJoints] = {3000, 1200, 1200};
int maxTPS[numJoints] = {150, 150, 150};
int maxAccel = 80;

ThreeAxisArmKinematics kinematics(130, 160, 142.9, 25.4); 

stepperMotor motors[numJoints] = {
  stepperMotor(driveEnablePin, dirPins[BASE], stepPins[BASE], 200, maxTPS[0], maxAccel),
  stepperMotor(driveEnablePin, dirPins[SHOULDER], stepPins[SHOULDER], 200, maxTPS[1], maxAccel),
  stepperMotor(driveEnablePin, dirPins[ELBOW], stepPins[ELBOW], 200, maxTPS[2], maxAccel)
};

int maxAngles[numJoints] = {170, 90, 90};

a4988DriveModule driveModules[numJoints] = {
  a4988DriveModule(motors[BASE], gearRatios[BASE], microStepRatio, maxAngles[BASE], limitPins[BASE], COUNTERCLOCKWISE), 
  a4988DriveModule(motors[SHOULDER], gearRatios[SHOULDER], microStepRatio, maxAngles[SHOULDER], limitPins[SHOULDER], CLOCKWISE), 
  a4988DriveModule(motors[ELBOW], gearRatios[ELBOW], microStepRatio, maxAngles[ELBOW], limitPins[ELBOW], COUNTERCLOCKWISE)
};

ThreeAxisArm arm(driveModules[BASE], driveModules[SHOULDER], driveModules[ELBOW], kinematics, driveEnablePin);



const float stepsPerRev[numJoints] = {
  motorStepsPerRev * microStepRatio * gearRatios[BASE],
  motorStepsPerRev * microStepRatio * gearRatios[SHOULDER],
  motorStepsPerRev * microStepRatio * gearRatios[ELBOW]
};

const float degreesPerStep[numJoints] = {360. / stepsPerRev[BASE], 360. / stepsPerRev[SHOULDER], 360. / stepsPerRev[ELBOW]};

int jointSteps[numJoints] = {0}; //initialize all values in array to 0
int jointSetSteps[numJoints] = {0};
double currentMicros = 0;

void setup() {
  for (int pin : dirPins) { pinMode(pin, OUTPUT); }
  for (int pin : stepPins) { pinMode(pin, OUTPUT); }
  pinMode(driveEnablePin, OUTPUT);
  digitalWrite(driveEnablePin, HIGH);
  for (int pin : limitPins) { pinMode(pin, INPUT_PULLUP); }

  driveModules[0].setAngleOffset(0);
  driveModules[1].setAngleOffset(25.4);
  driveModules[2].setAngleOffset(142.9);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Initialized");
  // arm.enableArm(true);
  // Serial.println("Enabled arm");
}

void loop() {
  if (Serial.available()) {
    printPositionExtrema();
    delay(10e4);
  }
}



// void loop() {
//   if (Serial.available()) {
//     float destination[3] = {50, 50, 90};
//     float destinationAngles[3];
//     kinematics.inverseKinematics(destination, destinationAngles);

//     Serial.print("Angles: ");
//     for (float angle: destinationAngles) Serial.print((String) angle + " ");
//     Serial.println("");

//     Serial.print("Angles: ");
//     float finalAngles[3];
//     kinematics.getAngles(finalAngles);
//     for (float angle: finalAngles) Serial.print((String) angle + " ");
//     Serial.println("");

//     Serial.println();
//     delay(100000);
//   }
// }

void printPositionExtrema() {
  for (int j=0; j<2; j++) {
    float test[3] {};
    float out[3];

    for (int i=0; i<3; i++) { 
      test[i] = j==0 ? driveModules[i].getMinAngle() : driveModules[i].getMaxAngle();
      // Serial.println((String) (j==0 ? driveModules[i].getMinAngle() : driveModules[i].getMaxAngle()));
    }

    kinematics.inverseKinematics(test, out);
    for (int i=1; i<4; i++) { 
      Serial.print(j==0 ? "[Min Position " : "[Max Position ");
      Serial.print((String) i + ": " + (String) out[i-1] + "] ");
    }
    Serial.println("");
  }
}