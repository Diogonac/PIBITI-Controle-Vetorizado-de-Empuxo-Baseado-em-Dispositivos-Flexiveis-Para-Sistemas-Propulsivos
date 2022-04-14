#ifndef pin_names_h
#define pin_names_h

#include "mbed.h"

// Servos
const PinName SERVO1 = D5; 
const PinName SERVO2 = D6; 

// I2C Comunications Pins
const PinName SDA = D14; // I2C1
const PinName SCL = D15; // I2C1

// UART Comunications Pins
const PinName SERIAL_TX1 = D1; 
const PinName SERIAL_RX1 = D0;

// Flags Alerts Pins
const PinName AMARELO = D9; 
const PinName VERDE = D10; 
const PinName VERMELHO = D8; 
const PinName BUZZER = D11; 

#endif