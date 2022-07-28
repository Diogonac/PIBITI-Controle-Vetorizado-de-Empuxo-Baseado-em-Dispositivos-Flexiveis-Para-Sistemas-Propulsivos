#include "imports.h"
#include "mbed.h"
#include <functional>

//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação TX, RX

// Imports
Actuators act;
Reference ref_gen;
Initialization init;
AttitudeObserver obs;
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
  obs.initIMU();

  // Definição da taxa de amostragem
  amostragem.attach(&callback_main, dt);

  input.attach(&callback_ref, dt_wave);

  // Arm the sistem to initialization
  init.arm();

  while (true) {
    if (flag1) {

      flag1 = false;

      // Input references
      ref_gen.ref_generator(theta_ref, phi_ref);

      // Control and OBS step
      obs.readIMU();
      obs.estimate(att_cont.f_x, att_cont.f_y, obs.Theta, obs.Q, obs.Phi, obs.P);
      att_cont.control(ref_gen.u_ref_phi[0], ref_gen.u_ref_theta[0], ref_gen.x_ref_phi, ref_gen.x_ref_theta, obs.estimated_phi, obs.estimated_theta);
      

      // Actuate signals
      act.actuate_servos(att_cont.f_x, att_cont.f_y, m * g * 1);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, m * g * 1);

      // Print my states
    //   pc.printf("STATE1= %f, STATE2= %f, STATE3=%f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2]);
    //   pc.printf("%f %f %f %f %f %f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2], att_cont.f_x, obs.Theta, obs.Q);
    //   pc.printf("%f %f %f\n\r", (180.0 * obs.estimated[0]  / pi), (180.0 * obs.estimated[1] / pi), (180.0 * obs.estimated[2] / pi));
    //   pc.printf("%f %f %f %f\n\r", att_cont.f_x, obs.estimated[0], obs.estimated[1], obs.estimated[2]);
      bt.printf("FX= %f, THETA= %f\n\r", (180.0 * obs.Phi / pi), (180.0 * obs.estimated_phi[0] / pi));
    //   pc.printf("%f %f %f %f %f\n\r", (180.0 * obs.Theta / pi), (180.0 * obs.estimated[0] / pi), (180.0 * obs.Q / pi), (180.0 * obs.estimated[1] / pi), (180.0 * obs.estimated[2] / pi));
    //   pc.printf("Q= %f, Q_hat= %f\n\r", (180.0 * obs.Q / pi), (180.0 * obs.estimated[1] / pi));
    //   pc.printf("%f %f\n\r", act.theta, (180.0 * obs.estimated_log[0] / pi));
    //   pc.printf("%f %f\n\r", (180.0 * obs.Q / pi), (180.0 * obs.q / pi));
    //   pc.printf("FX= %f\n\r", att_cont.f_x);
    //   pc.printf("%f %f %f %f %f %f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2], att_cont.f_x, obs.Theta, obs.Q);

    
      

      wave_input();
    }

    
  }
}

void callback_main() { flag1 = true; }

void callback_ref() { flag2 = !flag2; }

void wave_input() {
  if (flag2 == true) {
    theta_ref = 0.0;//(pi * 30) / 180;
    phi_ref = 0.0;
  }
  if (flag2 == false) {
    theta_ref = 0.0;//-(pi * 10)/180;
    phi_ref = 0.0;
  }
}
