#include "imports.h"
#include "mbed.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Imports
Actuators act;
AttitudeController att_cont;
AttitudeEstimator att_est;
Initialization init;
Reference ref_gen;

// Ticker para taxa de amostragem
Ticker amostragem;
Ticker input;


// Flag de segurança
bool flag1, flag2;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r, fx_ref;

void callback(void);
void wave_input(void);


int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  // Defino o valor das veriáveis de referência
  phi_r = 0.0;
  theta_r = 0.0;
  p_r = 0.0;
  q_r = 0.0;
  fx_ref = 0.0;
      
  flag2 = false;

  // Configuração dos periféricos
  act.servo_test(10.0, -10.0, 2.0);
  att_est.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

  input.attach(&wave_input, dt_wave);


  // Arm the sistem to initialization
  init.arm();

    while(true){
    if (flag1) {

      flag1 = false;
      att_est.estimate();
      att_cont.control(phi_r, theta_r, att_est.Phi, att_est.Theta, att_est.P, att_est.Q, p_r, q_r);

      act.actuate_servos(att_cont.f_x, att_cont.f_y, m * g * 0.5);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, m * g * 0.5);
      


    //   pc.printf("Wx= %f, Wy= %f\r\n", att_est.P, att_est.Q);
    //   pc.printf("Ftot=%f, FY=%f, FX=%f, PHI=%f, THETA=%f, phi_s= %f, theta_s= %f\r\n", act.total_thruster, att_cont.f_y, att_cont.f_x, att_est.Phi*180/pi, att_est.Theta*180/pi, act.phi_servo1, act.theta_servo2);
        //   pc.printf("%f %f\r\n", att_est.Theta * 180 / pi, att_cont.vel_erro);
    }
    if (flag2 == true) {

      //flag2 = false;
      theta_r = 0;//(pi * 10)/180;
    }
    if (flag2 == false){
    theta_r = 0;//-(pi * 10)/180;
    }
    pc.printf("%f %f\r\n", att_cont.f_x, att_est.Theta * 180 / pi);
    // pc.printf("%f %f\r\n", att_est.Q, att_est.q);

  }
}

void callback() { flag1 = true; }

void wave_input() { flag2 =! flag2; }




