#ifndef attitude_observer_h
#define attitude_observer_h

#include "mbed.h"
#include "imports.h"
#include "direct_one.h"
// Class observer
class AttitudeObserver

{
public:
  // Classe do construtor
  AttitudeObserver();

  // Main functions
  void initIMU();
  void readIMU(void);
  void estimate(double u, double theta_in, double Q_in, double phi_in, double P_in);

  // Auxiliar variables
  bool verifica_imu;
  double Phi, Theta, Psi;
  double P, Q, R, q;
  double estimated[3];



private:
  // My estimate functions
  DirectOne F2theta_hat;
  DirectOne F2d_theta_hat;
  DirectOne F2dd_theta_hat;

  DirectOne theta2theta_hat;
  DirectOne theta2d_theta_hat;
  DirectOne theta2dd_theta_hat;

  DirectOne d_theta2theta_hat;
  DirectOne d_theta2d_theta_hat;
  DirectOne d_theta2dd_theta_hat;

  BNO055 BNO055;

  int status_BNO055;        // Armazena o status do BNO055
  int status_sys_BNO055;    // Armazena o status do sistema do BNO055
  bool status_check_BNO055; // Armazena o status do BNO055
  int status_selftest;      // Armazena o valor do teste

};

#endif
