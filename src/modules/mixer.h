#ifndef mixer_h
#define mixer_h

#include "mbed.h"
#include "imports.h"


// Classe Mixer
class Mixer

{

public:

    // Construtor da classe
    Mixer();


    /* Aciona a válvula para entregar o empuxo (N) total desejado (baseado nos
    vetores empuxo) e os servos (graus) */
    void actuate(double f_x, double f_y, double f_z);
    


    void estado_seguro(void);

    void config_servos(void);

    double pos;

    double delta_phi[2];
    
    bool verifica_servos;

private:

    // Inicialização dos servos

    // Valvula PWM output
    PwmOut valvula;

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_amarelo;


    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    double tempo_servos, desloca_phi, desloca_theta;
    // Empuxo total (N)
    double empuxo_total;

    // Ângulos dos servos | phi --> roll(x) / theta --> pitch(y)
    double phi_servo1, theta_servo2, phi_total, theta_total;

    /* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
    da válvula e os ângulos desejados nos tamanhos de pulsos */
    void mixer(double f_x, double f_y, double f_z);

    // Converte o empuxo total em sinal para abertura da válvula
    double controle_valvula(double abertura_valvula);

    

};

#endif