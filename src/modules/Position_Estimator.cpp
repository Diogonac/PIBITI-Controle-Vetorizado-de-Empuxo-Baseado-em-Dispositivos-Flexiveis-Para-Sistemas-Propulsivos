#include "Position_Estimator.h"


PositionEstimator::PositionEstimator()
{

    // Initial values for velocity 
    Vx = 0.0;
    Vy = 0.0;
    Vz = 0.0;

    // Initial values for position 
    X = 0.0;
    Y = 0.0;
    Z = 0.0;

}


void PositionEstimator::get_pos(double liaX)
{

    Vx = Vx + liaX * dt;
    X = X + Vx * dt;

}
