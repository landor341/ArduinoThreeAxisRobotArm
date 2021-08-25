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

int maxTPS[numJoints] = {3000, 1200, 1200};

ThreeAxisArmKinematics kinematics(130, 160, 142.9, 25.4); 

stepperMotor motors[numJoints] = {
  stepperMotor(driveEnablePin, dirPins[BASE], stepPins[BASE], 200, 3000, 200),
  stepperMotor(driveEnablePin, dirPins[SHOULDER], stepPins[SHOULDER], 200, 100, 200),
  stepperMotor(driveEnablePin, dirPins[ELBOW], stepPins[ELBOW], 200, 100, 200)
};

int maxAngles[numJoints] = {170, 90, 90};

a4988DriveModule driveModules[numJoints] = {
  a4988DriveModule(motors[BASE], gearRatios[BASE], microStepRatio, maxAngles[BASE], limitPins[BASE], COUNTERCLOCKWISE), 
  a4988DriveModule(motors[SHOULDER], gearRatios[SHOULDER], microStepRatio, maxAngles[SHOULDER], limitPins[SHOULDER], CLOCKWISE), 
  a4988DriveModule(motors[ELBOW], gearRatios[ELBOW], microStepRatio, maxAngles[ELBOW], limitPins[ELBOW], COUNTERCLOCKWISE)
};

ThreeAxisArm arm(driveModules[BASE], driveModules[SHOULDER], driveModules[ELBOW], driveEnablePin);



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

  Serial.begin(9600);
  delay(1000);
  Serial.println("Initialized");
}


void loop() { //single joint testing
  digitalWrite(driveEnablePin, HIGH);
  if (Serial.available()) {
    digitalWrite(driveEnablePin, LOW);
    driveModules[BASE].enableMotor(true);

    driveModules[BASE].zero();
    Serial.println("Current Angle: " + (String) driveModules[BASE].getCurrentSteps());

    delay(3000);

    Serial.print("Parsed Int: ");
    int sentInt = Serial.parseInt();
    Serial.println((String) sentInt);

    driveModules[BASE].setPosition(90, ABSOLUTE, ANGLE);
    while (!driveModules[BASE].isAtRest()) {
      driveModules[BASE].update(micros());
      delayMicroseconds(200);
    }
    Serial.println("Current Angle: " + (String) driveModules[BASE].getCurrentSteps());

    digitalWrite(driveEnablePin, HIGH);
    delay(100000);
  }
}

// void loop() {
//   digitalWrite(driveEnablePin, HIGH);
//   if (Serial.available() > 0) {
//     digitalWrite(driveEnablePin, LOW);
//     for (int i = (sizeof(driveModules) / sizeof(driveModules[0])) - 1; i>=0; i--) {
//       driveModules[i].zero();
//       delay(1000);
//     }
//     digitalWrite(driveEnablePin, HIGH);
//     delay(50000);
//   }
// }


// loop() { //idek
  // currentMicros = micros();
  // driveModules[BASE].setPosition(round(driveModules[BASE].getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);
  // driveModules[SHOULDER].setPosition(round(driveModules[SHOULDER].getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);
  // driveModules[ELBOW].setPosition(round(driveModules[ELBOW].getCurrentAngle()) == 300 ? 0 : 300, ABSOLUTE);

  // double initialTime = currentMicros;
  // int i = 0;
  // while ((!driveModules[BASE].isAtRest() && !driveModules[SHOULDER].isAtRest() && !driveModules[ELBOW].isAtRest())) {
  //   currentMicros = micros();
  //   driveModules[BASE].update(currentMicros);
  //   driveModules[SHOULDER].update(currentMicros);
  //   driveModules[ELBOW].update(currentMicros);
  //   i++;
  // }
  // currentMicros = micros();
  // Serial.println((String) driveModules[BASE].currentPosition + "  " + (String) driveModules[BASE].desiredPosition);
  // Serial.println("cps of: " + (String) i + " per " + (String) ((currentMicros - initialTime)/1000000.) + " seconds");//(i / ((currentMicros - initialTime)/1000000.)));
// }
