/*
  ThreeAxisArmKinematics.cpp - Library for calculating 3 axis robot arm kinematics.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "a4988DriveModule.h"

a4988DriveModule::a4988DriveModule(stepperMotor motor, int gearRatio, int microStepsPerStep, float maxAngle, int limitPin, direction dirToSwitch)
:   enablePin(motor.enablePin),
    dirPin(motor.dirPin),
    stepPin(motor.stepPin),
    limitPin(limitPin),
    dirToSwitch(dirToSwitch),
    maxTPS(motor.maxTPS),
    acceleration(motor.maxTickAccel),
    ticksPerRevolution(motor.ticksPerRevolution * gearRatio / microStepsPerStep),
    maxTicks((int) degreesToTicks(maxAngle))
{
    setDir(motorClockwise);
} 
int a4988DriveModule::getCurrentSteps() { return currentSteps; }
float a4988DriveModule::getCurrentAngle() { return ticksToDegrees(currentSteps) + angleOffset; }
float a4988DriveModule::getCurrentVelocity() { return currentVelocity; }
void a4988DriveModule::setAngleOffset(float angleOffset) { angleOffset = angleOffset; }
void a4988DriveModule::enableMotor(boolean enable) { motorEnabled = enable; }

void a4988DriveModule::setDir(boolean clockwise) {
    digitalWrite(dirPin, clockwise);
    motorClockwise = clockwise;
}
boolean a4988DriveModule::atRest() { return currentSteps == setSteps; }
boolean a4988DriveModule::setPosition(float distance, positionMode posMode, motorUnits units) {
    if (posMode == ABSOLUTE) {
        if (units == TICKS) setSteps = (int) distance;
        else if (units == ANGLE) setSteps = (int) degreesToTicks(distance); 
    } else if (posMode == RELATIVE) {
        if (units == TICKS) setSteps = currentSteps + (int) distance;
        else if (units == ANGLE) setSteps = currentSteps + (int) degreesToTicks(distance);
    }

    if (setSteps >= maxTicks) {
        setSteps = currentSteps;
        return false;
    }
    setDir(setSteps > currentSteps);
    enableMotor(true);
    return true;
}
void a4988DriveModule::halt() {
    setSteps = currentSteps;
    currentVelocity = 0;
    enableMotor(false);
}
void a4988DriveModule::zero() {
    setDir(dirToSwitch);
    while (!digitalRead(limitPin)) {
        incrementMotor();
        delay(3);
    }
    currentSteps = 0;
    enableMotor(false);
}
void a4988DriveModule::update(double microsTime) {
    if (setSteps == currentSteps) {
        currentVelocity = 0;
        return;
    }
    double timeChange = (microsTime - lastIncrementTime);

    float ticksRemaining = setSteps - currentSteps;
    float minimumDistanceToZero = sq(currentVelocity) / (2 * acceleration);
    boolean deccelerate = minimumDistanceToZero >= ticksRemaining - 1;

    //calc current velocity
    currentVelocity = min(maxTPS, max(1, currentVelocity + (acceleration * (deccelerate ? -1 : 1))));

    //increment motor if needed and set time
    if (timeChange / 1000000  >= 1 / currentVelocity) { //if seconds passed is more than current seconds per tick
        incrementMotor();
        lastIncrementTime = microsTime;
    }
}


/* Private Methods */

boolean a4988DriveModule::incrementMotor() { //TODO: add limit testing
    if (motorEnabled && !digitalRead(limitPin) && currentSteps <= maxTicks) {
        if (motorClockwise) currentSteps++; //cw
        else currentSteps--; //ccw
        digitalWrite(stepPin,HIGH);
        // delayMicroseconds(1);
        digitalWrite(stepPin, LOW); //TODO: test for issues with timing
    }
}
float a4988DriveModule::ticksToDegrees(int ticks) { return currentSteps / (float) ticksPerRevolution * 360; }
int a4988DriveModule::degreesToTicks(float angle) { return (angle - angleOffset) / 360 * ticksPerRevolution; }
int a4988DriveModule::rotationsToTicks(float rotations) { return rotations * ticksPerRevolution; }