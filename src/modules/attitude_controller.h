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
       // Controlador regulador dos estados estimados
       double K_theta[3] = {-0.3210297713418580056732309913059, 0.10619473014635104346492511240285, 0.0};

       // Controlador PD
    //    double K_theta[3] = {8.5819, 5.3333, 0.0};


private:

    // MIMO Controller
    double controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]);

    // Saturation prevent
    double saturation(double u_signal);

    double ref_gain, u_control;
    double erro[3];
};

#endif








