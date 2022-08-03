#ifndef vertical_estimator_h
#define vertical_estimator_h

#include "mbed.h"
#include "imports.h"
#include "direct_one.h"

// Vertical observer Class
class VerticalEstimator {

public:
  // Class contructor
  VerticalEstimator();

  // Initialize class
  void init();

  // Estimate vertical position and velocity from model
  void estimate(double f_z, double z_m);

  // Correct vertical position and velocity with measurement
  void read(double phi , double theta);

  // Vertical position (m) and velocity (m/s) estimations
  double z, z_hat, z_dot_hat;
  double estimated_z[2];

private:

  // Range sensor object
  VL53L1X range;

  // My estimate functions
  DirectOne one2z_hat;
  DirectOne one2z_dot_hat;

  DirectOne Fz2z_hat;
  DirectOne Fz2z_dot_hat;

  DirectOne z2z_hat;
  DirectOne z2z_dot_hat;
  
};

#endif