#include "mbed.h"
#include "controlador_atitude.h"

// Classe do construtor
ControladorAtitude::ControladorAtitude()
{

    f_x = 0.0;
    f_y = 0.0;

}

/* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
void ControladorAtitude::controle(double phi_r, double theta_r, double phi, double theta, double p, double q, double p_r, double q_r)
{

    f_x = controlador_siso(theta_r, theta, q_r, q, KP, KD) * ((-1*I_yy)/l);

    f_y = controlador_siso(phi_r, phi, p_r, p, KP, KD) * (I_xx/l);

}


// Controlador siso
double ControladorAtitude::controlador_siso(double angulo_r, double angulo, double v_angular_r, double v_angular, double kp, double kd)
{

    e = (angulo_r - angulo);
    ce = (v_angular_r - v_angular);

    ganho_PID = kp * e + kd * ce;
    
    
    if(e < 0.0 & ce < 0) saida_fuzzy = e_min;
    
     if(e < 0.0 & ce == 0) saida_fuzzy = ce_min;
     
      if(e < 0.0 & ce > 0) saida_fuzzy = 0.0;
      
       if(e == 0.0 & ce < 0) saida_fuzzy = ce_min;
       
        if(e == 0.0 & ce == 0) saida_fuzzy = 0.0;
        
        
        
    
    
    
    variavel_SISO = ganho_PID;

    return variavel_SISO;

}



