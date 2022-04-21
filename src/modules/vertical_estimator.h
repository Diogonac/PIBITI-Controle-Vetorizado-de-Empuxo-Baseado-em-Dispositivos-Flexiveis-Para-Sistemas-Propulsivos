#ifndef vertical_estimator_h
#define vertical_estimator_h

#include "mbed.h"
#include "imports.h"

// Vertical estimator Class
class VerticalEstimator {

public:
  // Class contructor
  VerticalEstimator();

  // Initialize class
  void init();

  // Predict vertical position and velocity from model
  void predict(double f_z);

  // Correct vertical position and velocity with measurement
  void correct(double phi , double theta);

  // Vertical position (m) and velocity (m/s) estimations
  double z, w;

private:

  // Range sensor object
  VL53L0X range;
  
};

#endif