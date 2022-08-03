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
 void control(double u_r, double z_r[2], double z_hat[2]);


// Total thrust force (N)
 double f_z;
 double K_vert[2] = {5.406318853886616,4.000000000000000};

private:

    // MIMO Controller
    double controller(double u_r, double z_r[2], double z_hat[2], double K[2]);

    // Saturation prevent
    double saturation(double u_signal);

    double ref_gain, u_control;
    double erro[2];

};

#endif