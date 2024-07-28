#pragma once

#include "Parameters.h"
#include <Arduino.h>

enum class MOVE
{
    STOP,
    FWD,
    SS90ER,
    SS90EL
};

enum DIR
{
    RIGHT = -1,
    LEFT = 1
};

bool STOP(float *v_f0, float *theta_i0)
{
    *v_f0 = 0;
    *theta_i0 = 0;
    return false;
}

bool FWD(float *v_f0, float *theta_i0)
{
    // Вычисление времени
    static bool is_in_progress = false;
    static float time0 = 0;
    if (!is_in_progress)
    {
        time0 = millis() / 1000.0;
        is_in_progress = true;
    }
    const float time = millis() / 1000.0 - time0;

    // Математика
    *v_f0 = FORW_SPEED;
    *theta_i0 = 0;

    // Логика перехода
    if (time >= 1.8)
    // ЗАМЕНИТЕ 1.8 НА ПЕРЕСЧЕТ ПО СКОРОСТИ И РАЗМЕРУ ЯЧЕЙКИ
    {
        is_in_progress = false;
        return true;
    }
    return false;
}

bool SS90E(float *v_f0, float *theta_i0, DIR dir)
{
    // Вычисление времени
    static bool flag = false;
    static float time0 = 0;
    if (!flag)
    {
        time0 = millis() / 1000.0;
        flag = true;
    }
    const float time = millis() / 1000.0 - time0;

    // Математика
    *v_f0 = 0.1;

    if (time >= 0.225 && time < 1.285)
    {
        *theta_i0 = 1.48 * dir;
    }
    else
    {
        *theta_i0 = 0;
    }

    // Логика перехода
    if (time >= 1.51)
    {
        flag = 0;
        return true;
    }
    return false;
}
