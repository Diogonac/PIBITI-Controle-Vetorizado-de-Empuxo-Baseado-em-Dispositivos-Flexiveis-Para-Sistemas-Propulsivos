#ifndef estimador_atitude_h
#define estimador_atitude_h


#include "mbed.h"
#include "imports.h"


// Classe do estimador de atitude
class EstimadorAtitude
{
public:

    // Construtor da classe
    EstimadorAtitude();

    // Inicializa a IMU
    void config_imu();
    
    bool verifica_imu;

    // Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
    void estima();

    // Aplica o filtro Kalman na variavel de interesse
    double kalman_gx(double variavel_raw, double ruido_cov, double estima_cov);
    
    double kalman_gy(double variavel_raw, double ruido_cov, double estima_cov);

    // Ângulos de Euler de interesse
    double Phi, Theta, Psi;

    // Velocidades angular de interesse
    double P, Q, R;

    // Acelerações e velocidades angulares para o filtro
//    double ax, ay, az, gx, gy;

private:

    // Objeto da IMU
    BNO055 BNO055;
    

    
    int status_BNO055; //Armazena o status do BNO055
    int status_sys_BNO055; //Armazena o status do sistema do BNO055
    bool status_check_BNO055; //Armazena o status do BNO055
    int status_selftest; //Armazena o valor do teste
    
    // Variáveis para armazenar os offset dos ângulos de euler
    double offset_phi;
    double offset_theta;
    double offset_psi;
    
    // Variáveis para armazenar os offset das velocidades angulares
    double offset_gx;
    double offset_gy;
    double offset_gz;
    

    double erro_cov_gx; // Erro inicial das covariancias (deve ser 0)
    double erro_cov_gy; // Erro inicial das covariancias (deve ser 0)
    double erro_cov_ax; // Erro inicial das covariancias (deve ser 0)
    double erro_cov_ay; // Erro inicial das covariancias (deve ser 0)
    double erro_cov_az; // Erro inicial das covariancias (deve ser 0)


    double u_hat; // Valor inicial da variavel a ser filtrada

    double k;

    double k_gx; // Valor inical do ganho do filtro
    double k_gy; // Valor inical do ganho do filtro

    double k_ax; // Valor inical do ganho do filtro
    double k_ay; // Valor inical do ganho do filtro
    double k_az; // Valor inical do ganho do filtro

};

#endif