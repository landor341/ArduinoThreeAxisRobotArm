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
    maxVelocity((int) round(motor.maxVelocity)),
    acceleration((int) (motor.accelerationLimit * microStepsPerStep)),
    ticksPerRevolution((int) (motor.ticksPerRevolution * gearRatio * microStepsPerStep)),
    maxAngle(maxAngle),
    maxTicks(degreesToTicks(maxAngle))
{
    setDir(motorClockwise);
} 
int a4988DriveModule::getCurrentSteps() { return currentPosition; }
float a4988DriveModule::getCurrentAngle() { return ticksToDegrees(currentPosition) + angleOffset; }
float a4988DriveModule::getCurrentVelocity() { return currentVelocity; }
void a4988DriveModule::setAngleOffset(float offset) { angleOffset = offset; }
void a4988DriveModule::enableMotor(boolean enable) { motorEnabled = enable; }

void a4988DriveModule::setDir(boolean clockwise) {
    digitalWrite(dirPin, clockwise);
    motorClockwise = clockwise;
}
boolean a4988DriveModule::isAtRest() { return currentPosition == desiredPosition && currentVelocity == 0; }
boolean a4988DriveModule::setPosition(float distance, positionMode posMode, motorUnits units) {
    if (posMode == ABSOLUTE) {
        if (units == TICKS) desiredPosition = (int) distance;
        else if (units == ANGLE) desiredPosition = (int) degreesToTicks(distance - angleOffset); 
    } else if (posMode == RELATIVE) {
        if (units == TICKS) desiredPosition = currentPosition + (int) distance;
        else if (units == ANGLE) desiredPosition = currentPosition + (int) degreesToTicks(distance);
    } 

    if (desiredPosition > maxTicks || desiredPosition < 0) {
        desiredPosition = currentPosition;
        return false;
    }
    setDir(desiredPosition > currentPosition);
    enableMotor(true);
    return true;
}
boolean a4988DriveModule::setPosition(float distance, positionMode posMode) { return this->setPosition(distance, posMode, ANGLE); }
void a4988DriveModule::halt() {
    desiredPosition = currentPosition;
    currentVelocity = 0;
    enableMotor(false);
}
void a4988DriveModule::zero() {
    if (motorEnabled) {
        setDir(dirToSwitch);
        enableMotor(true);
        
        while (!atSwitch()) {
            enableMotor(true);
            incrementMotor();
            delay(2);
        }

        setPosition(0, ABSOLUTE);
        halt();
        
        currentPosition = 0;
        delay(10);

        setDir(!dirToSwitch);
        while (atSwitch()) {
            enableMotor(true);
            incrementMotor();
            delay(5);
        }

        halt();
    }
}
void a4988DriveModule::update(double microsTime) {
    if (desiredPosition == currentPosition) {
        currentVelocity = 0;
        return;
    }

    double timeChange = (microsTime - lastIncrementTime);

    float ticksRemaining = abs(desiredPosition - currentPosition);
    float minimumDistanceToZero = sq(currentVelocity) / (2 * acceleration);
    boolean deccelerate = minimumDistanceToZero >= ticksRemaining - 1;

    
    currentVelocity = min(maxVelocity, max(1, currentVelocity + (int) round(acceleration * (deccelerate ? -1 : 1))));

    if ((float) (timeChange / 1000000)  >= (1 / (float) currentVelocity)) { //if seconds passed is more than current seconds per tick
        incrementMotor();
        lastIncrementTime = microsTime;
    }
}


/* Private Methods */

boolean a4988DriveModule::incrementMotor() { //TODO: add limit testing
    if (motorEnabled) {
        if (motorClockwise) currentPosition++; //cw
        else currentPosition--; //ccw
        digitalWrite(stepPin,HIGH);
        // delayMicroseconds(1);
        digitalWrite(stepPin, LOW); //TODO: test for issues with timing
    }
    if ((atSwitch() && motorClockwise == dirToSwitch) || currentPosition > maxTicks) halt();
}
float a4988DriveModule::getMaxAngle() { return maxAngle; }
float a4988DriveModule::getMinAngle() { return angleOffset; }
float a4988DriveModule::ticksToDegrees(int ticks) { return currentPosition / (float) ticksPerRevolution * 360; }
int a4988DriveModule::degreesToTicks(float angle) { return (int) ((angle) * ticksPerRevolution / 360); }
int a4988DriveModule::rotationsToTicks(float rotations) { return rotations * ticksPerRevolution; }
int a4988DriveModule::ticksToRotations(int ticks) { return ticks / ticksPerRevolution; }
bool a4988DriveModule::atSwitch() { return !digitalRead(limitPin); }