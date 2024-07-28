#pragma once

#include <Arduino.h>

#include "Channels.h"
#include "EncoderDriver.h"
#include "ImuDriver.h"
#include "MotorDriver.h"
#include "Parameters.h"
#include "Tau.h"

void initTimer1()
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 10000;
    // WGM12 => CTC(Clear Timer on Compare Match), CS12 & CS10  => prescaler 1/1024
    TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);
    // OCIE1A => Timer1 compare match A interrupt
    TIMSK1 = bit(OCIE1A);
}

void setTimer1(float _time)
{
    long cnt = 16000000 / 1024 * _time; // cnt = clk / prescaler * time(s)
    if (cnt > 65535)
    {
        cnt = 65535; // "timer1 16bit counter over."
    }
    OCR1A = cnt; // Output Compare Register Timer1A
    TIMSK1 = bit(OCIE1A);
}

void stopTime1()
{
    TIMSK1 = 0;
}

void typerInit()
{
    noInterrupts();
    initTimer1();
    setTimer1(0.005);
    interrupts();
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)
{
    static uint32_t time0 = 0;
    const uint32_t dtime = micros() - time0;
    time0 = micros();

    ///////// SENSE /////////
    // Считывание датчиков
    encoderTick(); // 16us
    mpuTick();     // 1800us

    noInterrupts();
    //// Считывание каналов данных датчиков
    const float phi_L = G_phi_L;     // [rad]
    const float theta_i = G_theta_i; // [rad/s]
    //// Считывание каналов поступательной и угловой скорости
    const float v_f0 = G_v_f0;         // [m/s]
    const float theta_i0 = G_theta_i0; // [rad/s]
    interrupts();

    ///////// PLAN /////////
    // Расчет управляющих воздействий

    //// Вычислитель угла поворота робота ////
    static Integrator theta(Ts_typer_s);
    theta.tick(theta_i);

    //// Трансформатор поступательной скорости ////
    const float w_f0 = v_f0 / WHEEL_RADIUS; // [rad/s]

    //// Трансформатор угловой скорости ////
    const float w_Delta = theta_i0 * ROBOT_WIDTH / WHEEL_RADIUS; // [rad/s]

    //// Микшер ////
    const float w_L0 = w_f0 - w_Delta / 2; // [rad/s]
    const float w_R0 = w_f0 + w_Delta / 2; // [rad/s]

    //// Система управления скоростью мотора c ОС по энкодеру (левым) ////
    const float Tf = 2 * Ts_typer_s;            // [s]
    static VelEstimator w_Lest(Ts_typer_s, Tf); // [rad/s]
    w_Lest.tick(phi_L);

    // ПИ регулятор скорости
    const float err_wL = w_L0 - w_Lest.out; // [rad/s]

    const float K = 0.3;
    const float T = 0.2;
    const float Kp = K;
    const float Ki = K / T;

    static PIController pi_Lw(Ts_typer_s, Kp, Ki, SUPPLY_VOLTAGE); // [V]

    pi_Lw.tick(err_wL);

    const float u_L = pi_Lw.out; // [V]

    //// Вычислитель скорости мотора без энкодера (правый) ////
    const float w_Rest = w_Lest.out + G_theta_i * ROBOT_WIDTH / WHEEL_RADIUS;

    //// Система управления скоростью мотора c ОС по вычисленной скорости (правым) ////
    static PIController pi_Rw(Ts_typer_s, Kp, Ki, SUPPLY_VOLTAGE);

    const float err_wR = w_R0 - w_Rest;
    pi_Rw.tick(err_wR);

    const float u_R = pi_Rw.out;

    // 2200us
    ///////// ACT /////////
    // Приведение управляющих воздействий в действие и логирование данных
    motorTick(u_L, u_R);
    // 2230us

    // Serial.println(dtime);
}
