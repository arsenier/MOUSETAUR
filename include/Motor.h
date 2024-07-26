#pragma once

#include <Arduino.h>

#define SUPPLY_VOLTAGE 12.0

#define M1 4
#define E1 5
#define DIR1 1

#define E2 6
#define M2 7
#define DIR2 1

void motorInit()
{
    pinMode(M1, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E2, OUTPUT);
}

void motorPWM(int pinM, int pinE, int pwm)
{
    digitalWrite(pinM, pwm >= 0);
    analogWrite(pinE, abs(pwm));
}

void motorTick(float u_left_V, float u_right_V)
{
    const int16_t pwmL = 255.0 * constrain(u_left_V / SUPPLY_VOLTAGE, -1.0, 1.0) * DIR1;
    const int16_t pwmR = 255.0 * constrain(u_right_V / SUPPLY_VOLTAGE, -1.0, 1.0) * DIR2;

    motorPWM(M1, E1, pwmL);
    motorPWM(M2, E2, pwmR);
}

// void motorTick(float)
