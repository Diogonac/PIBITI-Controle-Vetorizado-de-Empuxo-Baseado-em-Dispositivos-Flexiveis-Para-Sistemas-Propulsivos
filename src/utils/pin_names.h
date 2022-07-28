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
const PinName SERIAL_TX1 = D8; 
const PinName SERIAL_RX1 = D2;

// Flags Alerts Pins
const PinName VERDE = D3; 
const PinName VERMELHO = D7;
const PinName BUZZER = D9;

// SPI pins
const PinName E_MOSI = D11;
const PinName E_MISO = D12;
const PinName E_SCK = D13;
const PinName E_CS1 = D10;

#endif