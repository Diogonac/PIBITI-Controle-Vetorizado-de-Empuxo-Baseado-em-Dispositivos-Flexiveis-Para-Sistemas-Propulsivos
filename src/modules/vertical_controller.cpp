#include "vertical_controller.h"
#include "math.h"
// Class Constructor
VerticalController::VerticalController()
{

// Initial conditions
f_z = 0.0;
uff = 0.0;
uout = 0.0;


}

// Control thrust force (N) given vertical position (m) and velocity (m/s)
void VerticalController :: control(double z_r , double z, double w)
{

f_z = m * (g + control_siso(z_r, z, w, kp_vert, kd_vert));

}

/* Control aceleration given reference position (m) and current position (m) and
velocity (m/s) with given controller gains */
double VerticalController :: control_siso(double pos_r , double pos , double vel , double kp, double kd)
{

return kp * (pos_r - pos) + kd * (0 - vel);

}

void VerticalController :: feed_foward(double ufb, double w, double f_max, double K1, double K2, double K3, double f_stiction, double f_coulomb, double z_r , double z){

    control(z_r, z, w);

    uff = f_stiction * (tanh(f_z/K1)-(f_z/f_max)) * (1 / cosh(w/K2)) + f_coulomb * tanh(w/K3);

    uout = uff + f_z;

}
