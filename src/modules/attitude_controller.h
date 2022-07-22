#ifndef attitude_controller_h
#define attitude_controller_h

#include "mbed.h"
#include "imports.h"

// Classe do controlador de atitude
class AttitudeController

{
public:

    // Classe do construtor
    AttitudeController();

    /* Controle das forças de empuxo (N) dado meu ângulo de referência (rad), ângulo
    atual (rad) e velocidade angular (rad/s) | p -> phi_ponto | q -> theta_ponto */
    void control(double u_phi_r, double u_theta_r, double phi_r[3], double theta_r[3], double phi_hat[3], double theta_hat[3]);

    // Auxiliar variables
    double f_x, f_y;
    double x_ref_gain, u_control;
    double xr_erro[3];

    // Controller gains
    // double K_theta[3] = {2.0736, 1.5915, 0.4389}; // Ts = 1.5s; Agressivo
    // double K_theta[3] = {0.413663671520527,0.580382289311921,0.264675130694548}; // Ts = 2.0s OS = 0.5%; Médio 
    // double K_theta[3] = {-0.438714207691537,-0.141811671931404,0.090415733134180}; // Ts = 3.0s OS = 0.5%; Bom
    // double K_theta[3] = {-0.177437956716218,0.112400602426246,0.160119492158327}; // Ts = 2.5s OS = 0.5%;; médio 
    // double K_theta[3] = {0.125220382790041,0.134157924895487,0.160119492158327}; // Ts = 2.5s OS = 2.0%;; Agressivo
    // double K_theta[3] = {-0.501177946478753,-0.173043541325012,0.090415733134180}; // Ts = 3.0s OS = 0.01%;; Ruím
    // double K_theta[3] = { 1.573847526004183,1.466526357478143,0.438934528254917}; // Ts = 1.5s OS = 0.01%;; Agressivo
    // double K_theta[3] = {0.202848553113672,0.510110583176302,0.264675130694548}; // Ts = 2.0s OS = 0.01%;; Ruim
    // double K_theta[3] = {-0.285375297340527,0.067426710499450,0.160119492158327}; // Ts = 2.5s OS = 0.01%;; Ruim 
    // double K_theta[3] = {-0.408495704262036,-0.126702420216654,0.090415733134180}; // Ts = 3.0s OS = 1.0%;; Bom 
    //    double K_theta[3] = {-0.633452497154889,-0.386080604277021,0.003286034353996}; // Ts = 4.0s OS = 1.0%;; Bom
    // double K_theta[3] = {0.499438018404615, 0.802681033438644, 0.438934528254917}; // Ts = 3.0s OS = 1.0%;; polo em 5 
    // double K_theta[3] = {0.077897361452241,0.315647709852334,0.189992531740105}; // Ts = 3.0s OS = 1.0%;; polo em 5 
    double K_theta[3] = {0.077897361452241,0.315647709852334,0.189992531740105}; // Ts = 2.0s OS = 1.0%;; Bom 








private:

    // MIMO Controller
    double controller(double u_r, double angle_r[3], double angle_hat[3], double K[3]);

    // Saturation prevent
    double saturation(double u_signal);

};

#endif








