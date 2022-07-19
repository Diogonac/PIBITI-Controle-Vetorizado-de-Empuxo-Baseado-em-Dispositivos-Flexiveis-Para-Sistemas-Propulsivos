#ifndef calibration_h
#define calibration_h

#include "mbed.h"
#include "imports.h"

// Classe Actuators
class Calibration

{

public:

    // Construtor da classe
    Calibration();

    // Desloca o servo e mede o ângulo do ponto central
    void calibra_servo_phi(void);
    void calibra_servo_theta(void);
    void servo_input_waves(double angle_ref, double KP);

    // Calibration test
    void test_calib(void);

    // Inicializa a IMU
    void config_calib_imu();

    /* DAC initialization */
    void config_dac();

    void calib_dac();

    // Used to attitude estimator
    // Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
    void estimate(void);

    // Set calibration true
    bool calibration;

    // Armazena os dados dos ângulos calibrados
    double phi_data_calib[11];
    double theta_data_calib[11]; 

    double voltageValve, DAC_out;

    bool verifica_imu, init_dac;

    // Ângulos de Euler de interesse
    double Phi, Theta, Psi;

    double erro_theta, erro_phi;

private:

    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    // DAC valve output
    MCP4725 valve;

    double lista_angulos_1[6] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
    double lista_angulos_2[6] = {0.0, -2.0, -4.0, -6.0, -8.0, -10.0};
    double lista_angulos[12] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 0.0, -2.0, -4.0, -6.0, -8.0, -10.0};
    double angle_calib_table[11] = {-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0};


    // Variáveis para calcular a média dos valores dos ângulos na calibração dos servos
    double mean_phi, mean_theta; // Média dos ângulos
    double sum_phi, sum_theta; // Somatória dos ângulos
    double estimated_phi, estimated_theta;

    double DAC[20] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0};


    // Check angles calibration
    void check_calibration (bool calib_phi, bool calib_theta);
    bool calibration_phi;
    bool calibration_theta;




    double angle_calib(double angle, double c1, double c2);
    double angle_calibrated;
    


    // Velocidades angular de interesse
    double P, Q, R;
    // Objeto da IMU
    BNO055 BNO055;

    int status_BNO055; //Armazena o status do BNO055
    int status_sys_BNO055; //Armazena o status do sistema do BNO055
    bool status_check_BNO055; //Armazena o status do BNO055
    int status_selftest; //Armazena o valor do teste

};

#endif


