#include <Arduino.h>

#include "Encoder.h"
#include "Motor.h"
#include "ImuDriver.h"
#include "Tau.h"
#include "Parameters.h"

#define Ts_us 5000               // Период квантования в [мкс]
#define Ts_s (Ts_us / 1000000.0) // Период квантования в [с]

void setup()
{
  Serial.begin(115200);

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

  const float phi_rad = enc_phi_rad;

  ///////// PLAN /////////
  // Расчет управляющих воздействий

  //// Задание поступательной скорости
  const float v_f0 = 0.2; // [м/с]
  const float w_f0 = v_f0 / WHEEL_RADIUS_M;

  //// Задание угловой скорости
  const float thetai_0 = 0;

  const float err_thetai = thetai_0 - phi_rad;

  static PIController pi_thetai(Ts_s, 1, 10, 20);
  pi_thetai.tick(err_thetai);
  const float dw = 0;

  //// Микшер
  const float w_l0 = w_f0 - dw / 2;
  const float w_r0 = w_f0 + dw / 2;

  //// Замкнутое управление ДПТ (левым) ////

  const float T_f_s = 2 * Ts_s;

  static VelEstimator w_est_rad_s(Ts_s, T_f_s);

  w_est_rad_s.tick(phi_rad);

  // ПИ регулятор скорости
  const float err_w_rad_s = w_l0 - w_est_rad_s.out;

  const float K = 1;
  const float T = 0.2;
  const float Kp = K;
  const float Ki = K / T;

  static PIController pi_w_V(Ts_s, Kp, Ki, SUPPLY_VOLTAGE);

  pi_w_V.tick(err_w_rad_s);

  const float u_l_V = pi_w_V.out;

  //// Разомкнутое управление ДПТ (правым) ////

  const float w_r_est = w_est_rad_s.out + gyro_z_rad_s * ROBOT_WIDTH_M / WHEEL_RADIUS_M;

  static PIController pi_w_r_V(Ts_s, Kp, Ki, SUPPLY_VOLTAGE);

  const float err_w_r_rad_s = w_r0 - w_r_est;
  pi_w_r_V.tick(err_w_r_rad_s);

  const float u_r_V = pi_w_r_V.out;

  ///////// ACT /////////
  // Приведение управляющих воздействий в действие и логирование данных
  motorTick(u_l_V, u_r_V);

  // Serial.print(dtime);
  Serial.print(" ");
  Serial.print(w_l0);
  Serial.print(" ");
  Serial.print(w_r0);
  Serial.print(" ");
  Serial.print(w_est_rad_s.out);
  Serial.print(" ");
  Serial.print(w_r_est);
  Serial.print(" ");
  Serial.print(u_l_V);
  Serial.print(" ");
  Serial.print(u_r_V);
  // Serial.print(" ");
  // Serial.print(gyro_z_raw_popugi);
  // Serial.print(" ");
  // Serial.print(gyro_z_rad_s);
  // Serial.print(" ");
  // Serial.print(gyro_z_angle_rad);
  Serial.println();
}
