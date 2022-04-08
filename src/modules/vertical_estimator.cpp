#include "vertical_estimator.h"
#include <cstdio>

// Class Constructor
VerticalEstimator::VerticalEstimator() : range(SDA, SCL)
{

// Initial conditions
z = 0.0;
w = 0.0;


}

// Initialize class
void VerticalEstimator ::init()
{

range.init();


}

// Predict vertical position and velocity from model
void VerticalEstimator :: predict(float f_t)
{
 z = z + w * dt;
 
 if(z > 0.05)
 {
 w = w + ((f_t/m)-g) * dt;
 }
 

}

// Correct vertical position and velocity with measurement
void VerticalEstimator :: correct(float phi , float theta)
{

range.read();

if(range.d < 2.0)
{
    //float z_m = cos(phi) * cos(theta) * range.d;
    float z_m = range.d;

    w = w + l1 * dt_range * (z_m - z);
    z = z_m;//+ l2 * dt_range * (z_m - z);
}

}