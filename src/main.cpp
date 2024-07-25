#include <Arduino.h>

#include "Encoder.h"
#include "Motor.h"

#define Ts_us 5000               // Период квантования в [мкс]
#define Ts_s (Ts_us / 1000000.0) // Период квантования в [с]

void setup()
{
  Serial.begin(115200);

  encoderInit();
  motorInit();
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
  encoderTick();
  const float phi_rad = enc_phi_rad;
  const float w0_rad_s = 4;

  ///////// PLAN /////////
  // Расчет управляющих воздействий
  const float T_f_s = 2 * Ts_s;

  static float I_w_est_rad = 0;
  const float err_w_est_rad = phi_rad - I_w_est_rad;
  const float w_est_rad_s = err_w_est_rad / T_f_s;

  const float dI_w_est_rad = w_est_rad_s * Ts_s;
  I_w_est_rad += dI_w_est_rad;

  // ПИ регулятор скорости
  const float err_w_rad_s = w0_rad_s - w_est_rad_s;

  const float K = 1;
  const float T = 0.1;
  const float Kp = K;
  const float Ki = K / T;

  const float p = err_w_rad_s * Kp;
  static float I = 0;
  const float i = I * Ki;
  const float u_V = p + i;

  if (u_V == constrain(u_V, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE) || (err_w_rad_s * u_V) < 0)
    I += err_w_rad_s * Ts_s;

  ///////// ACT /////////
  // Приведение управляющих воздействий в действие и логирование данных
  motorTick(u_V, 0);

  Serial.print(dtime);
  Serial.print(" ");
  Serial.print(w0_rad_s);
  Serial.print(" ");
  Serial.print(w_est_rad_s);
  Serial.println();
}
