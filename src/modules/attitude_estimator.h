#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"
#include "imports.h"
#include "direct_one.h"
// Class 
class AttitudeEstimator

{
public:
  // Classe do construtor
  AttitudeEstimator();

  // Main functions
  void init();
  void read(void);
  void estimate(double u_x, double u_y, double theta_in, double Q_in, double phi_in, double P_in);

  // Auxiliar variables
  bool verifica_imu;
  double Phi, Theta, Psi;
  double P, Q, R, q;
  double estimated_theta[3], estimated_phi[3];



private:
  // My estimate functions
  DirectOne F2theta_hat;
  DirectOne F2d_theta_hat;

  DirectOne theta2theta_hat;
  DirectOne theta2d_theta_hat;

  BNO055 BNO055;

  int status_BNO055;        // Armazena o status do BNO055
  int status_sys_BNO055;    // Armazena o status do sistema do BNO055
  bool status_check_BNO055; // Armazena o status do BNO055
  int status_selftest;      // Armazena o valor do teste

};

#endif
