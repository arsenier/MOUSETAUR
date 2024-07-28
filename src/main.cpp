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

float thetai_smooth_search(float time, float v_f0 = 0.1)
{
    const float l_forw = 0;

    const float time_mod = fmodf(time, 1.51);

    if (time_mod >= 0.225 && time_mod < 1.285)
    {
        return 1.48;
    }
    return 0;
}

void loop()
{
    ///////// TIMER /////////
    // Задание постоянной частоты главного цикла прогааммы
    static uint32_t timer = micros();
    // 4500us
    const uint32_t dtime = micros() - timer;
    while (micros() - timer < Ts_seq_us)
        ;
    timer = micros();

    ///////// SENSE /////////
    // Считывание датчиков

    // const float phi_L = G_phi_L;     // [rad]
    // const float theta_i = G_theta_i; // [rad/s]

    ///////// PLAN /////////
    // Расчет управляющих воздействий
    float v_f0 = 0;     // [m/s]
    float theta_i0 = 0; // [rad/s]

    const MOVE program[] = {F, R, L, F, S};
    static size_t programCounter = 0;

    bool isComplete = false;

    switch (program[programCounter])
    {
    case S:
        isComplete = STOP(&v_f0, &theta_i0);
        break;
    case F:
        isComplete = FWD(&v_f0, &theta_i0);
        break;
    case L:
        isComplete = SS90E(&v_f0, &theta_i0, LEFT);
        break;
    case R:
        isComplete = SS90E(&v_f0, &theta_i0, RIGHT);
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
