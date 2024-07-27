#include <Arduino.h>

#include "EncoderDriver.h"
#include "ImuDriver.h"
#include "MotorDriver.h"
#include "Parameters.h"
#include "Tau.h"

#define Ts_us 5000               // Период квантования в [мкс]
#define Ts_s (Ts_us / 1000000.0) // Период квантования в [с]

void setup()
{
    Serial.begin(115200);

    // Инициализация драйверов периферийных устройств
    encoderInit();
    motorInit();
    mpuInit();
}

void loop()
{
    ///////// TIMER /////////
    // Задание постоянной частоты главного цикла прогааммы
    static uint32_t timer = micros();
    const uint32_t dtime = micros() - timer;
    while (micros() - timer < Ts_us)
        ;
    timer = micros();

    ///////// SENSE /////////
    // Считывание датчиков
    encoderTick();
    mpuTick();

    const float phi_L = G_phi_L;     // [rad]
    const float theta_i = G_theta_i; // [rad/s]

    ///////// PLAN /////////
    // Расчет управляющих воздействий

    //// Вычислитель угла поворота робота ////
    static Integrator theta(Ts_s);
    theta.tick(theta_i);

    //// Задание поступательной и угловой скоростей ////
    const float v_f0 = 0.1;   // [m/s]
    const float theta_i0 = 0; // [rad/s]

    //// Трансформатор поступательной скорости ////
    const float w_f0 = v_f0 / WHEEL_RADIUS; // [rad/s]

    //// Трансформатор угловой скорости ////
    const float w_Delta = theta_i0 * ROBOT_WIDTH / WHEEL_RADIUS; // [rad/s]

    //// Микшер ////
    const float w_L0 = w_f0 - w_Delta / 2; // [rad/s]
    const float w_R0 = w_f0 + w_Delta / 2; // [rad/s]

    //// Система управления скоростью мотора c ОС по энкодеру (левым) ////
    const float Tf = 2 * Ts_s;            // [s]
    static VelEstimator w_Lest(Ts_s, Tf); // [rad/s]
    w_Lest.tick(phi_L);

    // ПИ регулятор скорости
    const float err_wL = w_L0 - w_Lest.out; // [rad/s]

    const float K = 0.3;
    const float T = 0.2;
    const float Kp = K;
    const float Ki = K / T;

    static PIController pi_Lw(Ts_s, Kp, Ki, SUPPLY_VOLTAGE); // [V]

    pi_Lw.tick(err_wL);

    const float u_L = pi_Lw.out; // [V]

    //// Вычислитель скорости мотора без энкодера (правый) ////
    const float w_Rest = w_Lest.out + G_theta_i * ROBOT_WIDTH / WHEEL_RADIUS;

    //// Система управления скоростью мотора c ОС по вычисленной скорости (правым) ////
    static PIController pi_Rw(Ts_s, Kp, Ki, SUPPLY_VOLTAGE);

    const float err_wR = w_R0 - w_Rest;
    pi_Rw.tick(err_wR);

    const float u_R = pi_Rw.out;

    ///////// ACT /////////
    // Приведение управляющих воздействий в действие и логирование данных
    motorTick(u_L, u_R);

    Serial.print(dtime);
    Serial.print(" ");
    Serial.print(w_L0);
    Serial.print(" ");
    Serial.print(w_R0);
    Serial.print(" ");
    Serial.print(w_Lest.out);
    Serial.print(" ");
    Serial.print(w_Rest);
    Serial.print(" ");
    Serial.print(u_L);
    Serial.print(" ");
    Serial.print(u_R);
    Serial.println();
}
