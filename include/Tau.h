#pragma once

#include <Arduino.h>

class Integrator
{
private:
    float I;
    float Ts;

public:
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

class VelEstimator
{
private:
    Integrator I;
    float Tf;
    float _out;

public:
    const float &out = _out;

    VelEstimator(float Ts, float _Tf)
        : I(Ts)
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

    PIController(float Ts, float _Kp, float _Ki, float _max_out)
        : I(Ts)
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

        if (u == constrain(u, -max_out, max_out) || (u * in) < 0)
            I.tick(in);

        _out = constrain(u, -max_out, max_out);
    }
};
