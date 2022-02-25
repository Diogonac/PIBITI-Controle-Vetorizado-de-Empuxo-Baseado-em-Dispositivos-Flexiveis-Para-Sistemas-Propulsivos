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
    void estimate(void);

    // Ângulos de Euler de interesse
    double Phi, Theta, Psi;

    // Velocidades angular de interesse
    double P, Q, R;

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

};

#endif