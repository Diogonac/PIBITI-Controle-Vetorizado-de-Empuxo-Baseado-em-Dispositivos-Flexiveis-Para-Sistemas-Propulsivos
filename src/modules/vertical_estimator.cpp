#include "vertical_estimator.h"

// Class Constructor
VerticalEstimator::VerticalEstimator()
    : range(SDA, SCL),
      one2z_hat(-0.000222237325, -0.00044447465, -0.000222237325, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0),
      one2z_dot_hat(-0.04893665896, -0.00897838793, 0.03995827103, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0),
      Fz2z_hat(0.00004530832314, 0.00009061664628, 0.00004530832314, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0),
      Fz2z_dot_hat(0.009976892755, 0.001830456255, -0.0081464365, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0),
      z2z_hat(0.09383353722, 0.00462144896, -0.08921208826, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0),
      z2z_dot_hat(0.462144896, 0, -0.462144896, 0.0, 1.0, -1.807711477, 0.8169543745, 0.0)
{

// Initial conditions
z = 0.0;
z_hat = 0.0;
z_dot_hat = 0.0;

}

// Initialize class
void VerticalEstimator ::init()
{
range.init();
}

// Estimate vertical position and velocity from model
void VerticalEstimator ::estimate(double f_z, double z_m) {

  if (z_m < 0.05) {
  z_hat = z_m;
  } else {
    z_hat = one2z_hat.update(1.0) + Fz2z_hat.update(f_z) + z2z_hat.update(z_m);
  }
  z_dot_hat = one2z_dot_hat.update(1.0) + Fz2z_dot_hat.update(f_z) + z2z_dot_hat.update(z_m);

  estimated_z[0] = z_hat;
  estimated_z[1] = z_dot_hat;

}

// Reatd vertical position and velocity with measurement
void VerticalEstimator :: read(double phi , double theta)
{

  range.read();
  z = cos(phi) * cos(theta) * range.d;

}