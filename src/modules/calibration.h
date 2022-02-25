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

    // Calibration test
    void phi_test_calib(void);
    void theta_test_calib(void);



    // Set calibration true
    bool calibration;


private:

    // Servos PWM outputs
    Servo servo1;
    Servo servo2;

    double lista_angulos_1[6] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
    double lista_angulos_2[6] = {0.0, -2.0, -4.0, -6.0, -8.0, -10.0};
    double lista_angulos[12] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 0.0, -2.0, -4.0, -6.0, -8.0, -10.0};


    // Variáveis para calcular a média dos valores dos ângulos na calibração dos servos
    double mean_phi, mean_theta; // Média dos ângulos
    double sum_phi, sum_theta; // Somatória dos ângulos
    double estimated_phi, estimated_theta;

    // Armazena os dados dos ângulos calibrados
    double phi_data_calib[11];
    double theta_data_calib[11]; 

    // Check angles calibration
    void check_calibration (bool calib_phi, bool calib_theta);
    bool calibration_phi;
    bool calibration_theta;


};

#endif


