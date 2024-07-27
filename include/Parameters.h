#pragma once

#define WHEEL_RADIUS (0.043 / 2) // [м]
#define ROBOT_WIDTH 0.08         // [м]
#define MOTOR_KE (4.0 / 42)      // [В/рад/с]

#define Ts_typer_us 2000               // Период квантования в [мкс]
#define Ts_typer_s (Ts_us / 1000000.0) // Период квантования в [с]

#define Ts_us 5000               // Период квантования в [мкс]
#define Ts_s (Ts_us / 1000000.0) // Период квантования в [с]
