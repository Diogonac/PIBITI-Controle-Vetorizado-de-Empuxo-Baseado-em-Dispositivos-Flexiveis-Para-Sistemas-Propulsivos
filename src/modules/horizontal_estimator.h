#ifndef horizontal_estimator_h
#define horizontal_estimator_h

#include "mbed.h"
#include "imports.h"
#include "direct_one.h"
// Class 
class HorizontalEstimator

{
public:
  // Classe do construtor
  HorizontalEstimator();

  // Main functions
  void init();
  void read(double phi, double theta, double p, double q, double z);
  void estimate(double phi, double theta, double f_x, double f_y);

  // Auxiliar variables
  bool verifica_imu;
  double x_hat, y_hat;
  double vel_x, vel_x_hat, vel_y, vel_y_hat;
  double estimated_x[2], estimated_y[2];



private:
  // My estimate functions

  
  // Flow sensor object
  PMW3901 flow;


};

#endif
