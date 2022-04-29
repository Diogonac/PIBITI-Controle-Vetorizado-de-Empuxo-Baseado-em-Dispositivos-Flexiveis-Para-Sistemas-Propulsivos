#include "vertical_estimator.h"
#include <cstdio>

// Class Constructor
VerticalEstimator::VerticalEstimator() : range(SDA, SCL)
{

// Initial conditions
z = 0.0;
z_m = 0.0;
w = 0.0;
w_verify = 0.0;
delta_z = 0.0;

}

// Initialize class
void VerticalEstimator ::init()
{

range.init();


}

// Predict vertical position and velocity from model
void VerticalEstimator :: predict(double f_z)
{
 z = z + w * dt;
 
//  if(z > 0.05)
//  {
//  w = w + ((f_z/m)-g) * dt;
//  }

}

// Correct vertical position and velocity with measurement
void VerticalEstimator :: correct(double phi , double theta)
{

range.read();

if(range.d < 2.0)
{
    //float z_m = cos(phi) * cos(theta) * range.d;
    z_m = range.d;
    delta_z = z_m - z;
    

    // w_verify = w_verify + l1 * dt_range * (z_m - z);
    // if (abs(w_verify) >= 0.02) {
    //     w = w + l1 * dt_range * (z_m - z);
    // }else {
    //     w = 0.001;
    // }
    // if (abs(delta_z) >= 0.005){
    w = w + l1 * dt_range * delta_z;
    z = z + l2 * dt_range * delta_z;
    // }
    // else{
    // w = w + l1 * dt_range * z_m;
    // z = z + l2 * dt_range * z_m;
    // }
}

}