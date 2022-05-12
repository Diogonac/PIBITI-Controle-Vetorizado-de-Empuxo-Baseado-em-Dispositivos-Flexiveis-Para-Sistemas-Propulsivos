#include "imports.h"
#include "mbed.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Imports
Actuators act;
AttitudeController att_cont;
AttitudeEstimator att_est;
Initialization init;

// Ticker para taxa de amostragem
Ticker amostragem;

// Flag de segurança
bool flag;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r;

void callback(void);

int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  // Defino o valor das veriáveis de referência
  phi_r = 0.0;
  theta_r = 0.0;
  p_r = 0.0;
  q_r = 0.0;

  // Configuração dos periféricos
  act.servo_test(105.0, 75.0, 1.0);
  att_est.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

  // Arm the sistem to initialization
  init.arm();

  while (abs(att_est.Phi) <= pi / 3.0 && abs(att_est.Theta) <= pi / 3.0 &&
         abs(att_est.P) <= 5.0 * pi && abs(att_est.Q) <= 5.0 * pi &&
         abs(att_est.R) <= 5.0 * pi && init.valve_flag == 1) {

    if (flag) {

      flag = false;

      att_est.estimate();

      att_cont.control(phi_r, theta_r, att_est.Phi, att_est.Theta, att_est.P, att_est.Q, p_r, q_r);

      act.actuate_servos(att_cont.f_x, att_cont.f_y, m * g * 0.3);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, m * g * 0.3);

    //   pc.printf("Wx= %f, Wy= %f\r\n", att_est.P, att_est.Q);
      pc.printf("Ftot=%f, FY=%f, FX=%f, PHI=%f, THETA=%f, phi_s= %f, theta_s= %f\r\n", act.total_thruster, att_cont.f_y, att_cont.f_x, att_est.Phi*180/pi, att_est.Theta*180/pi, act.phi_servo1, act.theta_servo2);
     
     
    }

  }

  act.safe_state();
  init.arm();
  att_est.estimate();
  wait_ms(10);
  pc.printf("Fora do while principal \r\n");
  pc.printf("PITCH= %f, ROLL= %f, P= %f, q= %f, r= %f\r\n", att_est.Theta,
            att_est.Phi, att_est.P, att_est.Q, att_est.R);
}
// }

void callback() { flag = true; }
