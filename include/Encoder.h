#pragma once

#include <Arduino.h>

#define ENC_A 8
#define ENC_B 9

#define ENC_DIR 1

#define ENC_PORT PINB
#define ENC_MASK 0b00110000
#define ENC_SHIFT 4

volatile int16_t counter = 0;
int8_t ett[4][4] = {0};

void encoderInit()
{
    // Отключение прерываний на время работы с регистрами
    noInterrupts();

    // Объявление пинов энкодера как входов
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);

    // Включение PCINT на порту B
    PCICR |= 0b00000001;

    // Активация пинов PCINT5 (D8) и PCINT6 (D9)
    PCMSK0 |= 0b00110000;

    // Настройка таблицы переходов
    ett[0b00][0b01] = -ENC_DIR;
    ett[0b01][0b11] = -ENC_DIR;
    ett[0b11][0b10] = -ENC_DIR;
    ett[0b10][0b00] = -ENC_DIR;

    ett[0b00][0b10] = ENC_DIR;
    ett[0b10][0b11] = ENC_DIR;
    ett[0b11][0b01] = ENC_DIR;
    ett[0b01][0b00] = ENC_DIR;

    // Включение прерываний для запуска энкодера
    interrupts();
}

ISR(PCINT0_vect)
{
    static int8_t enc_zn1 = 0;

    int8_t enc = (ENC_PORT & ENC_MASK) >> ENC_SHIFT;

    counter += ett[enc_zn1][enc];
    enc_zn1 = enc;
}
