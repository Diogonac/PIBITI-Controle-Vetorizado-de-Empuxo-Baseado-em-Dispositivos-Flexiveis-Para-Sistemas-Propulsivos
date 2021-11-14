#ifndef Position_Estimator_h
#define Position_Estimator_h

#include "mbed.h"
#include "imports.h"

// Classe Position contro
class PositionEstimator

{

public:

    // Construtor da classe
    PositionEstimator();

    /* Double integration to get the current position */
    void get_pos(double liaX);

    /** Values for current velocity **/
    double Vx, Vy, Vz;

    /** Values for current position **/
    double X, Y, Z;

private:


};

#endif