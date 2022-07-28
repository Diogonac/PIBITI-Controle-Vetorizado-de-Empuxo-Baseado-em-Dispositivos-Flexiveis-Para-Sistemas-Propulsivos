#ifndef vertical_observer_h
#define vertical_observer_h

#include "mbed.h"
#include "imports.h"

// Vertical observer Class
class VerticalObserver {

public:
  // Class contructor
  VerticalObserver();

  // Initialize class
  void init();

  // Predict vertical position and velocity from model
  void predict(double f_z);

  // Correct vertical position and velocity with measurement
  void correct(double phi , double theta);

  // Vertical position (m) and velocity (m/s) estimations
  double z, w, w_verify, delta_z, z_m;

private:

  // Range sensor object
  VL53L1X range;
  
};

#endif