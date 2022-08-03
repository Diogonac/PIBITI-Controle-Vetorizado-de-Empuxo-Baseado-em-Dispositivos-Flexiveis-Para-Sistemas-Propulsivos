#include "vertical_controller.h"
#include "math.h"

// Class Constructor
VerticalController::VerticalController()
{

// Initial conditions
f_z = 0.0;

}

// Control thrust force (N) given vertical position (m) and velocity (m/s)
void VerticalController :: control(double u_r, double z_r[2], double z_hat[2])
{

f_z = m * (g + controller(u_r, z_r, z_hat, K_vert));

}

/* Control aceleration given reference position (m) and current position (m) and
velocity (m/s) with given controller gains */
double VerticalController :: controller(double u_r, double z_r[2], double z_hat[2], double K[2])
{

  erro[0] = z_r[0] - z_hat[0];
  erro[1] = z_r[1] - z_hat[1];

  ref_gain = K[0]*erro[0] + K[1]*erro[1];

  u_control = ref_gain + u_r;
//   printf("%f %f %f \r\n", (180.0 * xr_erro[0] / pi), (180.0 * xr_erro[1] / pi), (180.0 * xr_erro[2] / pi));  
//   printf("%f %f\r\n", x_ref_gain, u_r);
  return u_control;

}
