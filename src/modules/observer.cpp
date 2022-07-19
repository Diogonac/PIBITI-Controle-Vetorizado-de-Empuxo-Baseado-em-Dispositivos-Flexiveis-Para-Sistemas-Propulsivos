#include "mbed.h"
#include "observer.h"

// Classe do construtor
Observer::Observer()
{

}

void Observer::simulation(double u, double theta, double d_theta){
  calculate(u_log, const1, const2, const3, const4);
  calculate(theta_log, const1, const5, const6, const7);
  calculate(d_theta_log, const1, const8, const9, const10);



  // Shifting positions
  state_hat[0][2] = state_hat[0][1];
  state_hat[0][1] = state_hat[0][0];
  state_hat[0][0] = estimated[0];

  state_hat[1][2] = state_hat[1][1];
  state_hat[1][1] = state_hat[1][0];
  state_hat[1][0] = estimated[1];

  state_hat[2][2] = state_hat[2][1];
  state_hat[2][1] = state_hat[2][0];
  state_hat[2][0] = estimated[2];

      
 
  
  
  
  

 
 // Logging the values     
    u_log[3] = u_log[2];
    u_log[2] = u_log[1];
    u_log[1] = u_log[0];
    u_log[0] = u;

    theta_log[3] = theta_log[2];
    theta_log[2] = theta_log[1];
    theta_log[1] = theta_log[0];
    theta_log[0] = theta;

    d_theta_log[3] = d_theta_log[2];
    d_theta_log[2] = d_theta_log[1];
    d_theta_log[1] = d_theta_log[0];
    d_theta_log[0] = d_theta;
    
    
    



  }



  



double Observer::calculate(double input_past[4], double num[3], double den1[4], double den2[4], double den3[4]){

  estimated[0] += num[0]*state_hat[0][0] + num[1]*state_hat[0][1] + num[2]*state_hat[0][2] + den1[0]*input_past[0] + den1[1]*input_past[1] + den1[2]*input_past[2] + den1[3]*input_past[3];
  estimated[1] += num[0]*state_hat[1][0] + num[1]*state_hat[1][1] + num[2]*state_hat[1][2] + den2[0]*input_past[0] + den2[1]*input_past[1] + den2[2]*input_past[2] + den2[3]*input_past[3];
  estimated[2] += num[0]*state_hat[2][0] + num[1]*state_hat[2][1] + num[2]*state_hat[2][2] + den3[0]*input_past[0] + den3[1]*input_past[1] + den3[2]*input_past[2] + den3[3]*input_past[3];

  return estimated;
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