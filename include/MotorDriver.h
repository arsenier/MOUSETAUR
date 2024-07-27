#pragma once

#include <Arduino.h>

#define SUPPLY_VOLTAGE 12.0    // [V]
#define MAX_OUTPUT_VOLTAGE 4.0 // [V]

#define M1 4
#define E1 5
#define DIR1 1

#define E2 6
#define M2 7
#define DIR2 1

/**
 * Инициализация драйвера мотора
 */
void motorInit()
{
    pinMode(M1, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E2, OUTPUT);
}

/**
 * Выдача ШИМ для мотора на данных пинах
 */
void motorPWM(int pinM, int pinE, int pwm)
{
    digitalWrite(pinM, pwm >= 0);
    analogWrite(pinE, abs(pwm));
}

/**
 * Обновление драйвера мотора
 */
void motorTick(float u_left_V, float u_right_V)
{
    // Рассчет требуемых значений ШИМ для получения необходимого напряжения на двигателе
    const int16_t pwmL =
        255.0 * constrain(constrain(u_left_V, -MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_VOLTAGE) / SUPPLY_VOLTAGE, -1.0, 1.0) *
        DIR1;
    const int16_t pwmR =
        255.0 * constrain(constrain(u_right_V, -MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_VOLTAGE) / SUPPLY_VOLTAGE, -1.0, 1.0) *
        DIR2;

    // Выдача вычисленного ШИМ сигнала на двигатели
    motorPWM(M1, E1, pwmL);
    motorPWM(M2, E2, pwmR);
}
