#pragma once

#define saturate(in, min, max) ((in) < (min) ? (min) : ((in) > (max) ? (max) : (in)))

/**
 * Интегрирующее звено
 *
 * Реализует численное интегрирование методом левых прямоугольников
 *
 * Реализуемая передаточная функция: `in/out = 1/s`
 */
class Integrator
{
  private:
    float I;  // Накопленное значение интеграла
    float Ts; // Период квантования системы

  public:
    // Ссылка на значение интеграла, доступная только на чтение
    const float &out = I;

    Integrator(float _Ts)
    {
        I = 0;
        Ts = _Ts;
    }

    void tick(float in)
    {
        const float dI = in * Ts;
        I += dI;
    }
};

/**
 * Вычислитель скорости на базе реального дифференцирующего звена
 *
 * Реализуемая передаточная функция: `in/out = s/(Tf*s + 1)`
 */
class VelEstimator
{
  private:
    Integrator I;
    float Tf;
    float _out;

  public:
    const float &out = _out;

    VelEstimator(float Ts, float _Tf) : I(Ts)
    {
        Tf = _Tf;
        _out = 0;
    }

    void tick(float in)
    {
        const float err = in - I.out;
        _out = err / Tf;
        I.tick(_out);
    }
};

/**
 * ПИ регулятор с защитой от насыщения условным интегрированием
 * и ограничением выходного сигнала
 *
 * Реализуемая передаточная функция: `in/out = Kp + Ki/s`
 */
class PIController
{
  private:
    Integrator I;
    float Kp;
    float Ki;
    float max_out;
    float _out;

  public:
    const float &out = _out;

    PIController(float Ts, float _Kp, float _Ki, float _max_out) : I(Ts)
    {
        Kp = _Kp;
        Ki = _Ki;
        max_out = _max_out;
        _out = 0;
    }

    void tick(float in)
    {
        const float p = in * Kp;
        const float i = I.out * Ki;
        const float u = p + i;

        if (u == saturate(u, -max_out, max_out) || (u * in) < 0)
            I.tick(in);

        _out = saturate(u, -max_out, max_out);
    }
};
