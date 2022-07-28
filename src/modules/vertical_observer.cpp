#include "vertical_observer.h"

// Class Constructor
VerticalObserver::VerticalObserver() : range(SDA, SCL)
{

// Initial conditions
z = 0.0;
z_m = 0.0;
w = 0.0;
w_verify = 0.0;
delta_z = 0.0;

}

// Initialize class
void VerticalObserver ::init()
{

range.init();


}

// Predict vertical position and velocity from model
void VerticalObserver :: predict(double f_z)
{
//  z = z + w * dt;
 
//  if(z > 0.05)
//  {
//  w = w + ((f_z/m)-g) * dt;
//  }

}

// Correct vertical position and velocity with measurement
void VerticalObserver :: correct(double phi , double theta)
{

  range.read();
  z = range.d;

// if(range.d < 2.0)
// {
//     float z_m = cos(phi) * cos(theta) * range.d;
 
//     w = w + l1 * dt_range * (z_m - z);
//     z = z + l2 * dt_range * (z_m - z);
// }

}