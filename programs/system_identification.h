#include "imports.h"
#include "mbed.h"

//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação com USB TX, RX

// Imports
Actuators act;
AttitudeController att_cont;
AttitudeEstimator obs;
Initialization init;

// Ticker para taxa de amostragem
Ticker amostragem;
Ticker input;
Timer impulse;

// Flag de segurança
bool flag1, flag2;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r, fx_ref, fy_ref, factor;

void callback(void);
void wave_input(void);

int main() {
  // Defino o valor das veriáveis de referência
  phi_r = 0.0;
  theta_r = 0.0;
  p_r = 0.0;
  q_r = 0.0;
  fx_ref = 0.0;
  fy_ref = 0.0;
  factor = 0.5;

  // Flags initialization
  flag1 = false;
  flag2 = false;

  // Configuração dos periféricos
  act.servo_test(10.0, -10.0, 2.0);
  obs.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

//   input.attach(&wave_input, dt_wave);

  // Arm the sistem to initialization
  init.arm();

  impulse.start();

  while (true) {
    if (flag1) {

      flag1 = false;
      obs.read();
      act.actuate_servos(fx_ref, fy_ref, m * g * factor);
      act.actuate_valve(fx_ref, fy_ref, m * g * factor);

      if (flag2 == false) {
        fx_ref = 1.3; //(pi * 10)/180;
        fy_ref = 1.3;
        factor = 0.8;

        if (impulse.read_ms()>=500) {
          flag2 = true;
          fx_ref = 0.0;
          fy_ref = 0.0;
          factor = 0.25;
        }
      }
      if (flag2 == true && impulse.read_ms()>=5500) {
        fx_ref = 1.3;
        fy_ref = 1.3;
        factor = 0.8;

        if (impulse.read_ms() >= 6000) {
          fx_ref = 0.0;
          fy_ref = 0.0;
          factor = 0.25;
        }
        if (impulse.read_ms() >= 11000) {
          impulse.reset();
          flag2 = false;
        }
      }

    //   if (flag2 == true) {

      //     fx_ref = 0.5; //(pi * 10)/180;
      //     fy_ref = 0.5;
      //   }
      //   if (flag2 == false) {
      //     fx_ref = -0.5;
      //     fy_ref = -0.5;
      //   }
      //   bt.printf("%f %f\r\n", act.voltageValve, obs.Theta * 180 / pi);
        bt.printf("%f %f %f\r\n", fx_ref, obs.Theta, obs.Q);
    //   bt.printf("FX=%f\r\n", fx_ref);
    }
  }
}

  void callback() { flag1 = true; }

//   void wave_input() { flag2 = !flag2; }