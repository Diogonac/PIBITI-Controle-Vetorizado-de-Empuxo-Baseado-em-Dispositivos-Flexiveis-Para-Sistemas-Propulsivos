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
    
    // Desloca o ponto central para 0º quando a condição limite for atingida
    void estado_seguro(void);

    // Verifica e configura o movimento dos servos  
    void config_servos(void);

    // Desloca o servo e mede o ângulo do ponto central
    void calibra_servo_MPU(void);

    // Confere a calibração do ângulo central
    void verifica_calib_servo_MPU(void);

    double theta_calib, phi_calib;

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
    double Phi_MPU_MM, Theta_MPU_MM, Psi_MPU_MM;
    double sum_phi, sum_theta;

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
    double lista_angulos[31] = {75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100.0, 101.0, 102.0, 103.0, 104.0, 105.0};   

    // Objeto da IMU
    MPU6050 MPU6050;

    double acc_MPU[3];
    
    
};

#endif