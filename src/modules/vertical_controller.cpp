#include "vertical_controller.h"
#include "math.h"

// Class Constructor
VerticalController::VerticalController()
{

// Initial conditions
f_z = 0.0;

pos_e_int = 0;


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

    float vel_e = 0 - vel;
    float pos_e = pos_r - pos;
    // pos_e_int += pos_e*dt;

    return kp * pos_e + kd * vel_e;

}
