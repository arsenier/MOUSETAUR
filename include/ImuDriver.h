#pragma once

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
#define BUFFER_SIZE 100
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define POPUGI_TO_RAD_S (250.0 / 32768 * DEG_TO_RAD)

int16_t gyro_z_raw_popugi = 0;
float gyro_z_rad_s = 0;

// ======= ФУНКЦИЯ КАЛИБРОВКИ =======
void calibration()
{
    long offsets[6];
    long offsetsOld[6];
    int16_t mpuGet[6];
    // используем стандартную точность
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    // обнуляем оффсеты
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    delay(10);
    for (byte n = 0; n < 10; n++)
    { // 10 итераций калибровки
        for (byte j = 0; j < 6; j++)
        { // обнуляем калибровочный массив
            offsets[j] = 0;
        }
        for (byte i = 0; i < 100 + BUFFER_SIZE; i++)
        {
            // делаем BUFFER_SIZE измерений для усреднения
            mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
            // пропускаем первые 99 измерений
            if (i >= 99)
            {
                for (byte j = 0; j < 6; j++)
                {
                    offsets[j] += (long)mpuGet[j]; // записываем в калибровочный массив
                }
            }
        }
        for (byte i = 0; i < 6; i++)
        {
            offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
            if (i == 2)
                offsets[i] += 16384; // если ось Z, калибруем в 16384
            offsetsOld[i] = offsets[i];
        }
        // ставим новые оффсеты
        mpu.setXAccelOffset(offsets[0] / 8);
        mpu.setYAccelOffset(offsets[1] / 8);
        mpu.setZAccelOffset(offsets[2] / 8);
        mpu.setXGyroOffset(offsets[3] / 4);
        mpu.setYGyroOffset(offsets[4] / 4);
        mpu.setZGyroOffset(offsets[5] / 4);
        delay(2);
    }
}

void mpuInit()
{
    Wire.begin();
    mpu.initialize();
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    calibration();
}

void mpuTick()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_z_raw_popugi = gx;

    gyro_z_rad_s = gyro_z_raw_popugi * POPUGI_TO_RAD_S;
}
