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

    void ref_generator(double input_signal1, double input_signal2);

    // Auxiliar variables
    double x_ref_theta[3];
    double u_ref_theta[1];

    double x_ref_phi[3];
    double u_ref_phi[1];
    

private:


};

#endif