#include "imports.h"
#include "mbed.h"
#include <functional>

//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação TX, RX

// Imports
Actuators act;
Reference ref_gen;
Initialization init;
AttitudeEstimator att_est;
AttitudeController att_cont;

// Ticker para taxa de amostragem
Ticker input;
Ticker amostragem;

// Flag de segurança
bool flag1, flag2;

// Variáveis de referencia
double phi_ref, theta_ref;

// Auxiliar functions
void wave_input(void);
void callback_main(void);
void callback_ref(void);

int main() {
  // Defino o valor das veriáveis de referência
  phi_ref = 0.0;
  theta_ref = 0.0;

  flag1 = false;
  flag2 = false;

  // Configuração dos periféricos
  act.servo_test(10.0, -10.0, 2.0);
  att_est.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback_main, dt);

//   input.attach(&callback_ref, dt_wave);

  // Arm the sistem to initialization
  init.arm();

  while (true) {
    if (flag1) {

      flag1 = false;

      // Input references
      ref_gen.ref_angle(theta_ref, phi_ref);

      // Control and OBS step
      att_est.read();
      att_est.estimate(att_cont.f_x, att_cont.f_y, att_est.Theta, att_est.Q, att_est.Phi, att_est.P);
      att_cont.control(ref_gen.u_ref_phi, ref_gen.u_ref_theta, ref_gen.ref_phi, ref_gen.ref_theta, att_est.estimated_phi, att_est.estimated_theta);
      
      // Actuate signals
      act.actuate_servos(att_cont.f_x, att_cont.f_y, m * g * 0.5);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, m * g * 0.5);

      // Print my states
    //   pc.printf("STATE1= %f, STATE2= %f, STATE3=%f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2]);
    //   pc.printf("%f %f %f %f %f %f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2], att_cont.f_x, obs.Theta, obs.Q);
    //   pc.printf("%f %f %f\n\r", (180.0 * obs.estimated[0]  / pi), (180.0 * obs.estimated[1] / pi), (180.0 * obs.estimated[2] / pi));
    //   pc.printf("%f %f %f %f\n\r", att_cont.f_x, obs.estimated[0], obs.estimated[1], obs.estimated[2]);
    //   bt.printf("FX= %f, THETA= %f\n\r", att_cont.f_x, (180.0 * obs.estimated_theta[0] / pi));
    //   bt.printf("FX= %f, THETA= %f\n\r", att_cont.f_x, (180.0 * att_est.Theta / pi));
    //   bt.printf("%f %f \n\r", theta_ref * 180.0 / pi, 180 * att_est.Theta / pi);
    //  bt.printf("%f %f %f\n\r", att_cont.f_x, 180 * att_est.Theta / pi, 180 * att_est.estimated_theta[0] / pi);
         bt.printf("%f\n\r",180 * att_est.estimated_theta[0] / pi);
    }
  }
}

void callback_main() { flag1 = true; }

void callback_ref() { flag2 = !flag2; }
