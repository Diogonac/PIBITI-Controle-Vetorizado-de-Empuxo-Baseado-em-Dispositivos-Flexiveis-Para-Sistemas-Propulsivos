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

    void calibra_servo_MPU(void);

    double pos;

    //void angulo_atual (double phi_atual, double theta_atual); // Coleta os ângulos apos a atuação dos servos
    
    double delta_angulos[2];

    bool verifica_servos;

        // Inicializa a IMU
    void config_MPU();

    bool verifica_MPU;

    // Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
    void estima_MPU();

    // Ângulos de Euler de interesse
    double Phi_MPU, Theta_MPU, Psi_MPU;

    // Acelerações e velocidades angulares para o filtro
    double ax, ay, az, gx, gy, gz;

private:

    // Inicialização dos servos

    // Valvula PWM output
    PwmOut valvula;

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_amarelo;


    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    double tempo_servos, desloca_phi, desloca_theta, delta_phi, delta_theta;
    // Empuxo total (N)
    double empuxo_total;

    // Ângulos dos servos | phi --> roll(x) / theta --> pitch(y)
    double phi_servo1, theta_servo2, phi_total, theta_total;

    /* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
    da válvula e os ângulos desejados nos tamanhos de pulsos */
    void mixer(double f_x, double f_y, double f_z);

    // Converte o empuxo total em sinal para abertura da válvula
    double controle_valvula(double abertura_valvula);

    // Lista de calibração
    double lista_angulos[16] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0};   

    // Objeto da IMU
    MPU6050 MPU6050;

    double acc_MPU[3];
    
};

#endif