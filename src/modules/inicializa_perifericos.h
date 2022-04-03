#ifndef inicializa_perifericos_h
#define inicializa_perifericos_h

#include "mbed.h"
#include "imports.h"


// Classe Mixer
class InicializaPerifericos

{

public:

    // Construtor da classe
    InicializaPerifericos();


    /* Chama as subfunções de inicialização dos perifericos */
    void verifica(bool estado_inicializacao_servos, bool estado_inicializacao_imu, bool estado_inicializacao_dac);
    
    bool sistema_verificado;
    
    void aviso_sonoro(void);


private:

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_verde;
    DigitalOut LED_vermelho;
    
    PwmOut buzzer;

};

#endif