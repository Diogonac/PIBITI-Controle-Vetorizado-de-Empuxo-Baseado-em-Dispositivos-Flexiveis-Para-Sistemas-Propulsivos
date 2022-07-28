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



    /* Aciona a válvula para entregar o empuxo (N) total desejado */
    void actuate_valve(double f_x, double f_y, double f_z);

    /* DAC initialization */
    void config_dac();

    // Aciona os servos para controlar a atitude do veiculo
    void actuate_servos(double f_x, double f_y, double f_z);
    
    // Desloca o ponto central para 0º quando a condição limite for atingida e desliga a propulsão
    void safe_state(void);

    // Teste the full servos fredoom
    void servo_test(double max_angle, double min_angle, double step_angle);

    // Initialization flags
    bool init_servos, init_dac;
    double voltageValve;
    
    // Ângulos dos servos | phi --> roll(x) / theta --> pitch(y)
    double phi_servo1, theta_servo2, phi, theta, total_thruster;


private:

    // DAC valve output
    MCP4725 valve;

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_vermelho;

    // Servos PWM outputs
    Servo servo1;
    Servo servo2;
    

    /* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
    da válvula e os ângulos desejados nos tamanhos de pulsos */
    void calc_thruster(double f_x, double f_y, double f_z);


    // Hold the angle position value if they were exceeded
    double clamp_angle(double angle, double maxAngle, double minAngle);

    // Convert the desired angle to clibrated desired angle
    double calibAngle(double angle, double angCoef, double linCoef);


    // Servos test positions 
    double pos, time;

};

#endif