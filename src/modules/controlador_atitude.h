#ifndef controlador_atitude_h
#define controlador_atitude_h

#include "mbed.h"
#include "imports.h"

// Classe do controlador de atitude
class ControladorAtitude

{
public:

    // Classe do construtor
    ControladorAtitude();

    /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
    atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
    void control(double phi_r, double theta_r, double phi, double theta, double p, double q, double p_r, double q_r);

    double f_x, f_y;

private:

// Variavel que armazena o empuxo
double variavel_SISO, e, ce;

    // Controlador siso
    double controlador_siso(double angulo_r, double angulo, double v_angular_r, double v_angular, double kp, double kd);
    double teste;
};

#endif








