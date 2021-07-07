/*
  ThreeAxisArmKinematics.cpp - Library for calculating 3 axis robot arm kinematics.
  Created by Landon R. Faris, June 2, 2021.
*/

//TODO: Deal with time function overflows

#include "Arduino.h"
#include "a4988DriveModule.h"




a4988DriveModule::a4988DriveModule(int enablePin, int dirPin, int stepPin, int motorTicksPerRevolution, int maxRPM, int RPMAcceleration, float maxAngle) {
    enablePin_ = enablePin;
    dirPin_ = dirPin;
    stepPin_ = stepPin;
    maxTPS_ = rotationsToTicks(maxRPM) / 60;
    TPSAcceleration_ = rotationsToTicks(RPMAcceleration) / 60;
    motorTicksPerRevolution_ = motorTicksPerRevolution;
    maxTicks_ = (int) degreesToTicks(maxAngle);

    setDir(motorClockwise);
} 
int a4988DriveModule::getCurrentSteps() { return currentSteps; }
float a4988DriveModule::getCurrentAngle() { return ticksToDegrees(currentSteps) + angleOffset_; }
float a4988DriveModule::getCurrentVelocity() { return currentVelocity; }
void a4988DriveModule::setAngleOffset(float angleOffset) { angleOffset_ = angleOffset; }
void a4988DriveModule::enableMotor(boolean enable) { motorEnabled = enable; }

void a4988DriveModule::setDir(boolean clockwise) {
    digitalWrite(dirPin_, clockwise);
    motorClockwise = clockwise;
}
boolean a4988DriveModule::atRest() { return currentSteps == setSteps; }
boolean a4988DriveModule::setPosition(float distance, bool isAbsolute, bool isTicks) {
    if (isAbsolute == true) {
        if (isTicks == true) setSteps = (int) distance;
        else if (isTicks == false) setSteps = (int) degreesToTicks(distance); 
    } else if (isAbsolute == false) {
        if (isTicks == true) setSteps = currentSteps + (int) distance;
        else if (isTicks == true) setSteps = currentSteps + (int) degreesToTicks(distance);
    }

    if (setSteps >= maxTicks_) {
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
void a4988DriveModule::zero(int limitSwitchpin, boolean clockwiseToSwitch) {
    setDir(clockwiseToSwitch);
    while (!digitalRead(limitSwitchpin)) {
        incrementMotor();
        delay(3);
    }
    currentSteps = 0;
}
void a4988DriveModule::update(double microsTime) {
    if (setSteps == currentSteps) {
        currentVelocity = 0;
        return;
    }
    double timeChange = (microsTime - lastIncrementTime);

    float ticksRemaining = setSteps - currentSteps;
    float minimumDistanceToZero = sq(currentVelocity) / (2 * TPSAcceleration_);
    boolean deccelerate = minimumDistanceToZero >= ticksRemaining - 1;

    //calc current velocity
    currentVelocity = min(maxTPS_, max(1, currentVelocity + (TPSAcceleration_ * (deccelerate ? -1 : 1))));

    //increment motor if needed and set time
    if (timeChange / 1000000  >= 1 / currentVelocity) { //if seconds passed is more than current seconds per tick
        incrementMotor();
        lastIncrementTime = microsTime;
    }
}


/* Private Methods */

boolean a4988DriveModule::incrementMotor() { //TODO: add limit testing
    if (motorEnabled && currentSteps >= 0 && currentSteps <= maxTicks_) {
        if (motorClockwise) currentSteps++; //cw
        else currentSteps--; //ccw
        digitalWrite(stepPin_,HIGH);
        // delayMicroseconds(1);
        digitalWrite(stepPin_, LOW); //TODO: test for issues with timing
    }
}
float a4988DriveModule::ticksToDegrees(int ticks) { return currentSteps / (float) motorTicksPerRevolution_ * 360; }
int a4988DriveModule::degreesToTicks(float angle) { return (angle - angleOffset_) / 360 * motorTicksPerRevolution_; }
int a4988DriveModule::rotationsToTicks(float rotations) { return rotations * motorTicksPerRevolution_; }