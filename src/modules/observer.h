#ifndef observer_h
#define observer_h

#include "mbed.h"
#include "imports.h"

// Class observer
class Observer

{
public:

    // Classe do construtor
    Observer();

    /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
    atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
    // void control(double u_phi_r, double u_theta_r, double phi_r, double theta_r, double p_r, double q_r, double dot_p_r, double dot_q_r, double phi_hat, double theta_hat, double p_hat, double q_hat, double dot_p_hat, double dot_q_hat);
    void simulation(double u, double theta, double d_theta);
    double theta_log[4] = {0, 0, 0, 0};
    double d_theta_log[4] = {0, 0, 0, 0};
    double u_log[4] = {0, 0, 0, 0};

  private:
    
  double calculate(double input_past[4], double num[3], double den1[4], double den2[4], double den3[4]);

    double x_ref_gain, u_control;
    double xr_erro[3];
    double state_hat[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double estimated[3] = {0, 0, 0};

    
;

};

#endif








