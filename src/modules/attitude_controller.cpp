#include "mbed.h"
#include "attitude_controller.h"

// Classe do construtor
AttitudeController::AttitudeController()
{

    f_x = 0.0;
    f_y = 0.0;
    pos_erro = 0.0;
    pos_e_int = 0.0;
    printf("K1= %f, K2= %f\r\n", K1, K2);

}

/* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
void AttitudeController::control(double phi_r, double theta_r, double phi, double theta, double p, double q, double p_r, double q_r)
{

    f_x = controlador_siso(theta_r, theta, q, K1, K2) * (I_yy/l); 

    f_y = controlador_siso(phi_r, phi, p, -K1, -K2) * (-I_xx/l);

    //printf("%f %f\r\n", f_x, f_y);

}

// Controlador siso
double AttitudeController::controlador_siso(double angulo_r, double angulo, double v_angular, double kp, double kd)
{

    pos_erro = angulo_r - angulo;
    pos_e_int += pos_erro*dt;

    return kp * pos_erro -kd * v_angular + pos_e_int * 8.0;

}