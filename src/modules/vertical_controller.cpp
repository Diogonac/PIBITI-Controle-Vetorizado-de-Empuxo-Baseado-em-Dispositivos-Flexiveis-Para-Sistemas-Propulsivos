#include "vertical_controller.h"
#include "math.h"
// Class Constructor
VerticalController::VerticalController()
{

// Initial conditions
f_z = 0.0;
uff = 0.0;
uout = 0.0;


}

// Control thrust force (N) given vertical position (m) and velocity (m/s)
void VerticalController :: control(double z_r , double z, double w)
{

f_z = m * (g + control_siso(z_r, z, w, kp_vert, kd_vert));

}

/* Control aceleration given reference position (m) and current position (m) and
velocity (m/s) with given controller gains */
double VerticalController :: control_siso(double pos_r , double pos , double vel , double kp, double kd)
{

return kp * (pos_r - pos) + kd * (0 - vel);

}

void VerticalController :: feed_foward(double ufb, double w, double z_r , double z){

    // Tentativa 1
    //uff = (w/abs(w)) * (viscous * abs(w) + f_coulomb); 

    // Tentativa 2
    // if(abs(w) > 0.1){
    //     f_coulomb = f_coulomb*0.5; 
    // }
    // uff = (w/abs(w)) * (viscous * abs(w) + f_coulomb); 
    
    // Tentativa 3
    // uff = viscous * tanh(w/K1) + F_coulomb * tanh(w/K1); 


    // Tentativa 4
    uff = viscous*tanh(w/K1) + F_coulomb*(1/cosh(w/K1))*tanh(w/K1);

    // Tentativa 5
    // uff = (w/abs(w)) * viscous * abs(w) + F_coulomb*(1/cosh(w/K1))*tanh(w/K1);

    /* A determinação deste sinal está provocando muito ruido quando 
    o sistema esta com velocidades muito baixas; 
    Tentar habilitar a predição da
    velocidade 
    Fazer o cálculo do sinal com menos casas decimais*/
    // uff = f_stiction * (tanh(f_z/K1)-(f_z/f_max)) * (1 / cosh(w/K2)) + f_coulomb * tanh(w/K3);
    uout = uff + ufb;

}