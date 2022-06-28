#include "mbed.h"
#include "gravity_feed_foward.h"

GravityFeedFoward::GravityFeedFoward()
{
    p_x = 0.0;
    
    p_y = 0.0;

    p_z = 0.0;
}

void GravityFeedFoward::gravityFF(double phi, double theta, double psi, double P)
{

    p_x =  P * sin(theta);

    p_y = -P * cos(theta) * sin(phi);

    p_z = -P * cos(phi) * cos(theta);

}
