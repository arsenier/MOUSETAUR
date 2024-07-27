#pragma once

#include <Arduino.h>

#define ENC_A 8
#define ENC_B 9

#define ENC_DIR -1

#define ENC_PORT PINB
#define ENC_MASK 0b00110000
#define ENC_SHIFT 4

#define ENC_TICK_PER_REV 12                                    // [ticks]
#define GEAR_RATIO 100                                         // [1]
#define TICK_TO_RAD (TWO_PI / (ENC_TICK_PER_REV * GEAR_RATIO)) // [rad/tick]

volatile int16_t counter = 0; // [ticks]
int8_t ett[4][4] = {0};

volatile float G_phi_L = 0; // [rad]

/**
 * Инициализация драйвера энкодера
 */
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

/**
 * Обработчик прерываний энкодера
 */
ISR(PCINT0_vect)
{
    // Буфер для хранения фазы энкодера в предыдущей итерации
    static int8_t enc_zn1 = 0;

    // Считывание текущей фазы энкодера
    int8_t enc = (ENC_PORT & ENC_MASK) >> ENC_SHIFT;

    // Инкремент счетчика согласно таблице переходов
    counter += ett[enc_zn1][enc];
    // Сохранение фазы энкодера
    enc_zn1 = enc;
}

/**
 * Обновление значения угла поворота вала двигателя
 */
void encoderTick()
{
    // Отключение прерываний для чтения 16 битного числа
    noInterrupts();
    int16_t counter_buff = counter;
    counter = 0;
    interrupts();

    // Обновление сигнала текущего угла поворота энкодера
    G_phi_L += counter_buff * TICK_TO_RAD;
}
