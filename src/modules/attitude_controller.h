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
    double x_ref_gain, u_control;
    double xr_erro[3];

    // Controller gains
    double K_theta[3] = {-0.019197938977378,0.999171875401519,0.116692792111049}; // Ts = 1.0s OS = 0.5%;; Bom 

private:

    // MIMO Controller
    double controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]);

    // Saturation prevent
    double saturation(double u_signal);

};

#endif








