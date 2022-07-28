#ifndef initialization_h
#define initialization_h

#include "mbed.h"
#include "imports.h"


// Classe Mixer
class Initialization

{

public:

    // Construtor da classe
    Initialization();


    /* Chama as subfunções de inicialização dos perifericos */
    void verify(bool estado_inicializacao_servos, bool estado_inicializacao_imu, bool estado_inicializacao_dac);
    
    bool sistema_verificado, valve_flag;
    
    void arm(void);


private:

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_verde;
    DigitalOut LED_vermelho;
    
    PwmOut buzzer;

};

#endif