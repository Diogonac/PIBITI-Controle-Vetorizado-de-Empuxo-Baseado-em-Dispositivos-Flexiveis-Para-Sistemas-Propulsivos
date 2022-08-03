#ifndef reference_h
#define reference_h

#include "mbed.h"
#include "imports.h"

// Classe Reference Generator
class Reference

{

public:

    // Construtor da classe
    Reference();

    void ref_angle(double input_signal1, double input_signal2);
    void ref_vertical(double input_signal1);


    // Auxiliar variables
    double ref_theta[3], ref_phi[3], ref_z[3];
    double u_ref_theta, u_ref_phi, u_ref_z;
    

private:


};

#endif