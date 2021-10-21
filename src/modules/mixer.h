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

    // Inicializa a IMU
    void config_MPU();

    // Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
    void estima_MPU();

    // Variável para receber os valores dos ângulos calibrados
    double theta_calib, phi_calib;

    // Define o incremento na inicialização dos servos
    double pos;

    // Armazena os dados de deslocamento dos servos para calcular o tempo de espera
    double delta_angulos[2];

    // Boleanas para verificar a inicialização nominal do MPU6050 e dos servos
    bool verifica_servos, verifica_MPU;

    // Vaiaveis para estimar os ângulos de Euler
    double Phi_MPU_a, Theta_MPU_a, Phi_MPU_g, Theta_MPU_g, Psi_MPU_g; // Ângulos estimados sem filtro
    double Phi_MPU, Theta_MPU, Psi_MPU; // Ângulos estimados com filtro complementar 
    double p_bias, q_bias, r_bias; // Erro sistemático da integração numérica
    double p, q, r; // Velocidades angular com a correção do erro sistemático 
    double sum_p, sum_q, sum_r; // somatória para o cálculo do erro sistemático

    // Variáveis para calcular a média dos valores dos ângulos na calibração dos servos
    double Phi_MPU_MM, Theta_MPU_MM; // Média dos ângulos
    double sum_phi, sum_theta; // Somatória dos ângulos 
    
    // Acelerações e velocidades angulares para o filtro
    double ax, ay, az, gx, gy, gz;

private:

    // Valvula PWM output
    PwmOut valvula;

    // LED Amarelo, indicador de saída de loop
    DigitalOut LED_amarelo;

    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    // Objeto da IMU
    MPU6050 MPU6050;

    /* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
    da válvula e os ângulos desejados nos tamanhos de pulsos */
    void mixer(double f_x, double f_y, double f_z);

    // Converte o empuxo total em sinal para abertura da válvula
    double controle_valvula(double abertura_valvula);

    // Garante que nenhum valor fora do intervalo [75 - 105] será impresso nos servos
    double seguranca_servos(double angulo);

    // Lista com os ângulos que os servos serão calibrados
    // double lista_angulos[31] = {75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100.0, 101.0, 102.0, 103.0, 104.0, 105.0};   
    //double lista_angulos[31] = {-15.0, -14.0, -13.0, -12.0, -11.0, -10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0};  
    double lista_angulos[17] = {-16.0, -14.0, -12.0, -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0};

    // Variáveis para calcular o deslocamento dos servos
    double tempo_servos, desloca_phi, desloca_theta, delta_phi, delta_theta;

    // Empuxo total (N)
    double empuxo_total;

    // Ângulos dos servos | phi --> roll(x) / theta --> pitch(y)
    double phi_servo1, theta_servo2, phi_total, theta_total;

    // Armazena os dados do MPU6050 
    double acc_MPU[3];
    double gyr_MPU[3];
    
};

#endif