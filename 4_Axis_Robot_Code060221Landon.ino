/*
 * motor controller reference: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
 * arduino cam possibility: https://circuitdigest.com/microcontroller-projects/how-to-use-ov7670-camera-module-with-arduino
 * 
 *                                        Questions or ideas
 * The motor has 200 steps per rev, what is the step mode of the motor controller (looks like quarter step)
 * Which joint is which?
 * Limit switches for minning out non-base joints? (possible to do with voltage readings)
 * make another inverse kinematics that allows you to pass a vector  
 * I think the kinematics assumes the second joint is at z=0, negative z's might work but it's confusing
 *  
 */

#include "src/ThreeAxisArmKinematics/ThreeAxisArmKinematics.h"
#include "src/a4988DriveModule/a4988DriveModule.h"
#include "src/ThreeAxisArm/ThreeAxisArm.h"

const int numJoints = 3;

const int dirPins[numJoints] = {5, 6, 7};
const int stepPins[numJoints] = {2, 3, 4};
const int driveEnablePin=8;

const int StepsPerRev=200; //steps on the actual stepper motor
const int microStepRatio = 4; //4 microsteps / 1 full step
const float gearRatios[3] = {5, 4.25, 4.25}; //5 motor rotations / 1 joint


const float revSteps = StepsPerRev * microStepRatio * gearRatio;
const float degPerStep = 360. / revSteps;

int jointSteps[numJoints] = {0}; //initialize all values in array to 0
int jointSetSteps[numJoints] = {0};

unsigned long previousMillis = 0;
double currentMicros = 0;
const int interval = 800;
unsigned long Heartbeat = 0;

ThreeAxisArmKinematics kinematics(130, 160, 142.9, 25.4); 

a4988DriveModule joint0Motor(driveEnablePin, dirPins[0], stepPins[0], revSteps, 1, 2, 350); 
a4988DriveModule joint1Motor(driveEnablePin, dirPins[1], stepPins[1], revSteps, 1, 2, 350); 
a4988DriveModule joint2Motor(driveEnablePin, dirPins[2], stepPins[2], revSteps, 1, 2, 350); 

ThreeAxisArm arm();

void setup() {
  // Serial.begin(9600);
  // delay(500);
  // Serial.println("here!");
  // delay(500);
  joint0Motor.enableMotor(true);
  joint1Motor.enableMotor(true);
  joint2Motor.enableMotor(true);

  // delay(500);
  // Serial.println("motors enabled");
  // delay(500);
}

void loop() {
  // delay(1000);
  // Serial.println("Starting");
  currentMicros = micros();
  joint0Motor.setPosition(round(joint0Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);
  joint1Motor.setPosition(round(joint1Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);
  joint2Motor.setPosition(round(joint2Motor.getCurrentAngle()) == 300 ? 0 : 300, true, false);
  // Serial.println("Position Set");
  double initialTime = currentMicros;
  int i = 0;
//  joint0Motor.currentSteps = 160;
  while ((!joint0Motor.atRest() && !joint1Motor.atRest() && !joint2Motor.atRest())) {
    currentMicros = micros();
    joint0Motor.update(currentMicros);
//    joint1Motor.update(currentMicros);
//    joint2Motor.update(currentMicros);
    i++;
  }
  currentMicros = micros();
  // Serial.println((String) joint0Motor.currentSteps + "  " + (String) joint0Motor.setSteps);
  // Serial.println("cps of: " + (String) i + " per " + (String) ((currentMicros - initialTime)/1000000.) + " seconds");//(i / ((currentMicros - initialTime)/1000000.)));
}
