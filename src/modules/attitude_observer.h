#ifndef attitude_observer_h
#define attitude_observer_h

#include "imports.h"
#include "mbed.h"

// Class observer
class AttitudeObserver

{
public:
  // Classe do construtor
  AttitudeObserver();

  // Main functions
  void initIMU();
  void readIMU(void);
  void estimate(double u);

  // Auxiliar variables
  bool verifica_imu;
  double Phi, Theta, Psi;
  double P, Q, R;

  double theta_log[4];
  double d_theta_log[4];
  double u_log[4];
  double estimated[3];


private:
  BNO055 BNO055;

  int status_BNO055;        // Armazena o status do BNO055
  int status_sys_BNO055;    // Armazena o status do sistema do BNO055
  bool status_check_BNO055; // Armazena o status do BNO055
  int status_selftest;      // Armazena o valor do teste

  void calculate(double input_past[4], double num[3], double den1[4], double den2[4], double den3[4]);

  double x_ref_gain, u_control;
  double xr_erro[3];
  double state_hat[3][3];

  // Observer
  double const1[3] = {-2.233, 1.662, -0.4121};
  double const2[4] = {2.863e-8, 8.589e-8, 8.589e-8, 2.863e-8};
  double const3[4] = {0.0002546, 0.0003198, -0.0001244, -0.0001895};
  double const4[4] = {0.06498, -0.02024, -0.05779, 0.02744};
  double const5[4] = {0.1279, -0.06252, -0.1196, 0.07082};
  double const6[4] = {-0.006747, 0.004597, 0.006644, -0.004701};
  double const7[4] = {-0.2238, 0.1433, 0.218, -0.149};
  double const8[4] = {0.004273, -0.002048, -0.003991, 0.002331};
  double const9[4] = {0.2261, -0.1361, -0.2178, 0.1443};
  double const10[4] = {2.474, -1.86, -2.479, 1.855};
};

#endif
