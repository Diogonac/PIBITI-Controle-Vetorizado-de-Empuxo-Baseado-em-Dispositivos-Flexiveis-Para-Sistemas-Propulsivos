#ifndef estimador_atitude_MPU6050_h
#define estimador_atitude_MPU6050_h


#include "mbed.h"
#include "imports.h"


// Classe do estimador de atitude
class EstimadorAtitudeMPU
{
public:

    // Construtor da classe
    EstimadorAtitudeMPU();

    // Inicializa a IMU
    void config_MPU();

    bool verifica_MPU;

    // Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
    void estima_MPU();

    // Ângulos de Euler de interesse
    double Phi_MPU, Theta_MPU, Psi_MPU;

    // Acelerações e velocidades angulares para o filtro
    double ax, ay, az, gx, gy, gz;

    void calibra_angulo(int tempo, double incremento);

    double pos, of_1, of_2;

private:

    // Objeto da IMU
    MPU6050 MPU6050;
    Servo servo1;
    Servo servo2;

//    // Variáveis para armazenar os offset dos ângulos de euler
//    double offset_phi;
//    double offset_theta;
//    double offset_psi;
//
//    // Variáveis para armazenar os offset das velocidades angulares
//    double offset_gx;
//    double offset_gy;
//    double offset_gz;

    double acc_MPU[3];


};

#endif