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
    // void control(double u_phi_r, double u_theta_r, double phi_r, double theta_r, double p_r, double q_r, double dot_p_r, double dot_q_r, double phi_hat, double theta_hat, double p_hat, double q_hat, double dot_p_hat, double dot_q_hat);
    void control(double u_phi_r, double u_theta_r, double phi_r[3], double theta_r[3], double phi_hat[3], double theta_hat[3], double K_phi[3], double K_theta[3]);

    double f_x, f_y;

private:

    // Controlador MIMO
    double controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]);

    double x_ref_gain, u_control;
    double xr_erro[3];

};

#endif








