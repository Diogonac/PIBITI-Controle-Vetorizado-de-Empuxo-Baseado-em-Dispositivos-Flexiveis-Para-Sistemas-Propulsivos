#ifndef pin_names_h
#define pin_names_h

#include "mbed.h"

// Pinos dos atuadores
const PinName VALVULA = D3;
const PinName SERVO1 = D5; // Antigo D4
const PinName SERVO2 = D6; // Antigo D2


// Pinos de comumicação I2C
const PinName IMU_SDA = D14; 
const PinName IMU_SCL = D15;


// Pinos de comumicação UART
const PinName SERIAL_TX1 = D1; 
const PinName SERIAL_RX1 = D0;

// Pinos dos indicadores
const PinName AMARELO = D8; //Mudar para D8
const PinName VERDE = D9; //Mudar para D9
const PinName VERMELHO = D10; //Mudar para D10
const PinName BUZZER = D11; //Mudar para D11

#endif