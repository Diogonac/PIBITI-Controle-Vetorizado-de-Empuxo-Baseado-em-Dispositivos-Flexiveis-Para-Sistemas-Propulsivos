#include "mbed.h"
#include "attitude_controller.h"
#include <cstdio>

// Classe do construtor
AttitudeController::AttitudeController()
{

    f_x = 0.0;
    f_y = 0.0;
    u_control = 0.0;
    ref_gain = 0.0;

    // printf("K1= %f, K2= %f\r\n", K1, K2);
}

void AttitudeController::control(double u_phi_r, double u_theta_r, double phi_r[3], double theta_r[3], double phi_hat[3], double theta_hat[3]){

  
  f_x = saturation(controller(u_theta_r, theta_r, theta_hat, K_theta));
  f_y = saturation(controller(u_phi_r, phi_r, phi_hat, K_theta));

}

double AttitudeController::controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]){

  erro[0] = angle_r[0] - angle_hat[0];
  erro[1] = angle_r[1] - angle_hat[1];
  erro[2] = angle_r[2] - angle_hat[2];

  ref_gain = K[0]*erro[0] + K[1]*erro[1] + K[2]*erro[2];// + erro_1*10.0;

  u_control = ref_gain + u_r;// - 1.0;
//   printf("%f %f %f \r\n", (180.0 * xr_erro[0] / pi), (180.0 * xr_erro[1] / pi), (180.0 * xr_erro[2] / pi));  
//   printf("%f\r\n", erro_1);
  return u_control;
    
}

double AttitudeController::saturation(double u_signal) {

  if (u_signal > Fsat) {
    u_signal = Fsat;
  }

  if (u_signal < -Fsat) {
    u_signal = -Fsat;
  }

  return u_signal;
}