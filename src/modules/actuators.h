#ifndef actuators_h
#define actuators_h

#include "mbed.h"
#include "imports.h"

// Classe Actuators
class Actuators

{

public:

    // Construtor da classe
    Actuators();

    /* Aciona a válvula para entregar o empuxo (N) total desejado (baseado nos
    vetores empuxo) e os servos (graus) */
    void actuate_servos(double f_x, double f_y, double f_z);

    // Converte o empuxo total em sinal para abertura da válvula
    double actuate_valve(double abertura_valvula);
    
    // Desloca o ponto central para 0º quando a condição limite for atingida e desliga a propulsão
    void safe_state(void);

    // Teste the full servos fredoom
    void servo_test(double max_angle, double min_angle, double step_angle);

    // Convert the desired angle to clibrated desired angle
    double PHI(double phi_angle);
    double THETA(double theta_angle);

    bool init_servos;

private:

    // Valvula PWM output
    PwmOut valvula;

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_amarelo;

    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    /* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
    da válvula e os ângulos desejados nos tamanhos de pulsos */
    void calc_thruster(double f_x, double f_y, double f_z);

    // Garante que nenhum valor fora do intervalo [75 - 105] será impresso nos servos
    double safe_angle(double angulo);

    // Ângulos dos servos | phi --> roll(x) / theta --> pitch(y)
    double phi_servo1, theta_servo2, phi, theta, total_thruster;

    // Servos test positions 
    double pos, time;
};

#endif