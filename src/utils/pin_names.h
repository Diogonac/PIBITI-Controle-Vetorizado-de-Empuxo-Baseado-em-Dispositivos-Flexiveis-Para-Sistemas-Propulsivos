#ifndef pin_names_h
#define pin_names_h

#include "mbed.h"

// Pinos dos atuadores
const PinName VALVULA = D4;
const PinName SERVO1 = D5;
const PinName SERVO2 = D6;


// Pinos de comumicação I2C
//const PinName IMU_SDA = D14; 
//const PinName IMU_SCL = D15;
//
//const PinName MPU_SDA = PB_3; 
//const PinName MPU_SCL = PB_10;

const PinName IMU_SDA = D0; 
const PinName IMU_SCL = D1;

const PinName MPU_SDA = D14; 
const PinName MPU_SCL = D15;


// Pinos de comumicação UART
const PinName SERIAL_TX1 = D1; 
const PinName SERIAL_RX1 = D0;

// Pinos dos indicadores
const PinName AMARELO = D7;
const PinName VERDE = D8;
const PinName VERMELHO = D9;
const PinName BUZZER = D10;

#endif