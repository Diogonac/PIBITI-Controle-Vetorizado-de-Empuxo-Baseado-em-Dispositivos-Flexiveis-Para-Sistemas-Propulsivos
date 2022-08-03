#ifndef attitude_controller_h
#define attitude_controller_h

#include "mbed.h"
#include "imports.h"

// Classe do controlador de atitude
class AttitudeController

{
public:

    // Classe do construtor
    AttitudeController();

    /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
    atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
    void control(double u_phi_r, double u_theta_r, double phi_r[3], double theta_r[3], double phi_hat[3], double theta_hat[3]);

    // Auxiliar variables
    double f_x, f_y;


    // Controller gains
    double K_theta[3] = {6.1374614350561387254856526851654, 157.74609133969096319560776464641, 10.96}; // Ts = 1.0s OS = 0.5%;; Bom 

double erro_1;

private:

    // MIMO Controller
    double controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]);

    // Saturation prevent
    double saturation(double u_signal);

    double ref_gain, u_control;
    double erro[3];
};

#endif








