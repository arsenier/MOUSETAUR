#include <Arduino.h>

#include "EncoderDriver.h"
#include "ImuDriver.h"
#include "MotorDriver.h"
#include "Parameters.h"
#include "Profiles.h"
#include "Tau.h"
#include "Typer.h"

void setup()
{
    Serial.begin(115200);

    // Инициализация драйверов периферийных устройств
    encoderInit();
    motorInit();
    mpuInit();

    typerInit();
}

void loop()
{
    ///////// TIMER /////////
    // Задание постоянной частоты главного цикла прогааммы
    static uint32_t timer = micros();
    while (micros() - timer < Ts_us)
        ;
    const uint32_t dtime = micros() - timer;
    timer = micros();

    ///////// SENSE /////////
    // Считывание датчиков

    noInterrupts();
    const float phi_L = G_phi_L;     // [rad]
    const float theta_i = G_theta_i; // [rad/s]
    interrupts();

    ///////// PLAN /////////
    // Расчет управляющих воздействий

    //// Вычислитель угла поворота робота ////
    static Integrator theta(Ts_s);
    theta.tick(theta_i);

    //// Задатчик поступательной и угловой скоростей (профиля движения) ////

    float v_f0 = 0.1; // [m/s]
    float theta_i0;   // [rad/s]

    const MOVE program[] = {L, L, R, F, R, R, F, F, R, S};
    static size_t programCounter = 0;

    bool isComplete = false;

    switch (program[programCounter])
    {
    case S:
        isComplete = STOP(&v_f0, &theta_i0);
        break;
    case F:
        isComplete = FWD(&v_f0, &theta_i0, 0);
        break;
    case R:
        isComplete = SS90E(&v_f0, &theta_i0, RIGHT);
        break;
    case L:
        isComplete = SS90E(&v_f0, &theta_i0, LEFT);
        break;

    default:
        break;
    }
    if (isComplete)
    {
        programCounter++;
        if (programCounter >= (sizeof(program) / sizeof(program[0])))
        {
            programCounter = sizeof(program) / sizeof(program[0]);
        }
    }

    ///////// ACT /////////
    // Приведение управляющих воздействий в действие и логирование данных

    noInterrupts();
    G_v_f0 = v_f0;
    G_theta_i0 = theta_i0;
    interrupts();

    Serial.print(dtime);
    // Serial.print(" ");
    // Serial.print(w_L0);
    // Serial.print(" ");
    // Serial.print(w_R0);
    // Serial.print(" ");
    // Serial.print(w_Lest.out);
    // Serial.print(" ");
    // Serial.print(w_Rest);
    // Serial.print(" ");
    // Serial.print(u_L);
    // Serial.print(" ");
    // Serial.print(u_R);
    Serial.println();
}
