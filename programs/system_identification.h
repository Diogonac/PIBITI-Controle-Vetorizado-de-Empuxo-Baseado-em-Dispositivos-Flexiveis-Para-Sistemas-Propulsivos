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
Ticker input;

// Flag de segurança
bool flag1, flag2;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r, fx_ref, fy_ref;

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
  fy_ref = 0.0;

  // Flags initialization
  flag1 = false;
  flag2 = false;

  // Configuração dos periféricos
  act.servo_test(10.0, -10.0, 2.0);
  att_est.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

  input.attach(&wave_input, dt_wave);

  // Arm the sistem to initialization
  init.arm();

  while (true) {
    if (flag1) {

      flag1 = false;
      att_est.estimate();
      att_cont.control(phi_r, theta_r, att_est.Phi, att_est.Theta, att_est.P, att_est.Q, p_r, q_r);
      act.actuate_servos(fx_ref, fy_ref, m * g * 0.1);
      act.actuate_valve(fx_ref, fy_ref, m * g * 0.1);

      if (flag2 == true) {

        fx_ref = 0.5; //(pi * 10)/180;
        fy_ref = 0.5;
      }
      if (flag2 == false) {
        fx_ref = -0.5;
        fy_ref = -0.5;
      }
      // pc.printf("%f %f\r\n", theta_r, att_est.Theta * 180 / pi);
      pc.printf("%f %f\r\n", fy_ref, att_est.Theta * 180 / pi);
    }
  }
}

  void callback() { flag1 = true; }

  void wave_input() { flag2 = !flag2; }