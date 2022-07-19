#include "mbed.h"
#include "attitude_controller.h"

// Classe do construtor
AttitudeController::AttitudeController()
{

    f_x = 0.0;
    f_y = 0.0;

    x_ref_gain = 0.0;
    
    pos_erro = 0.0;
    vel_erro = 0.0;
    pos_erro_past = 0.0;
    vel_erro_past = 0.0;
    u_control = 0.0;

    proportional = 0.0;
    derivative = 0.0;
    integrator = 0.0;
    integrator_past = 0.0;

    // printf("K1= %f, K2= %f\r\n", K1, K2);
}

void AttitudeController::control(double u_phi_r, double u_theta_r, double phi_r[3], double theta_r[3], double phi_hat[3], double theta_hat[3], double K_phi[3], double K_theta[3]){

  f_x = controller(u_theta_r, theta_r, theta_hat, K_theta);
  f_y = controller(u_phi_r, phi_r, phi_hat, K_phi);

}

double AttitudeController::controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]){

  xr_erro[0] = angle_r[0] - angle_hat[0];
  xr_erro[1] = angle_r[1] - angle_hat[1];
  xr_erro[2] = angle_r[2] - angle_hat[2];

  x_ref_gain = K[0]*xr_erro[0] + K[1]*xr_erro[1] + K[2]*xr_erro[2];

  u_control = x_ref_gain + u_r;

  return u_control;
    
}

































// /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
// atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
// void AttitudeController::control(double phi_r, double theta_r, double phi, double theta, double p, double q, double p_r, double q_r)
// {

  
//     f_x = controller(theta_r, theta, q, K1, K2, Ki);// * (I_yy/l);

//     f_y = controller(phi_r, phi, p, K1, K2, Ki);// * (I_yy/l);

//     //printf("%f %f\r\n", f_x, f_y);

// }

// // Controlador siso
// double AttitudeController::controller(double angulo_r, double angulo, double v_angular, double K1, double K2, double Ki)
// {

//     // Valores atuais
//     pos_erro = angulo_r - angulo;
//     vel_erro = 0 - v_angular;

//     // u_control = 1.5199 * pos_erro + 0.5598 * vel_erro + 2.4412 * pos_erro_int;

//     proportional = K1 * pos_erro;
//     derivative = K2 * vel_erro;

//     if (abs(u_control) <= 1.8) {
//       integrator = integrator + Ki * dt * 0.5 * (pos_erro + pos_erro_past);
//       integrator_past = integrator;
//     } else {
//       integrator = integrator_past;
//     }

//     pos_erro_past = pos_erro;
//     vel_erro_past = vel_erro;

//     u_control = proportional + derivative + integrator;

//     return u_control * K;
//     // return pos_erro*5.4063 + vel_erro*4.0;

// }