#ifndef vertical_controller_h
#define vertical_controller_h

#include "mbed.h"
#include "imports.h"

// Vertical controller Class
class VerticalController {

public:
  // Class contructor
  VerticalController();

/* Control total thrust force (N) given reference vertical position (m) and
current vertical position (m) and velocity (m/s) */
 void control(double z_r , double z, double w);


// Total thrust force (N)
double f_z;

private:

/* Control aceleration given reference position (m) and current position (m)
and velocity (m/s) with given controller gains */
double control_siso(double pos_r , double pos , double vel , double kp , double kd);

float pos_e_int;
  
};

#endif