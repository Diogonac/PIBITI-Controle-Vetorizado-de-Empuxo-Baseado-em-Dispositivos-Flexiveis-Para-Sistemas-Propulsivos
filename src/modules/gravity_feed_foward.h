/* This file was used only for attitude dynamics tests */

#ifndef gravity_feed_foward_h
#define gravity_feed_foward_h

#include "mbed.h"
#include "imports.h"

class GravityFeedFoward
{
public:

    // Class constructor
    GravityFeedFoward();

    /* Based on euler angles and gravity force, this function return gravity forces contributions on body coordinate frame */
    void gravityFF(double phi, double theta, double psi, double P);

    double p_x, p_y, p_z;

private:

};

#endif








