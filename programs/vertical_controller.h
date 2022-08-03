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
VerticalEstimator ver_est;
VerticalController ver_cont;

// Ticker para taxa de amostragem
Ticker input;
Ticker amostragem;
Ticker range_sample;

// Flag de segurança
bool flag1, flag2, flag3;

// Variáveis de referencia
double phi_ref, theta_ref, vertical_ref;

// Auxiliar functions
void wave_input(void);
void callback_main(void);
void callback_range(void);
void callback_ref(void);

int main() {
  // Defino o valor das veriáveis de referência
  phi_ref = 0.0;
  theta_ref = 0.0;
  vertical_ref = 0.15;

  flag1 = false;
  flag2 = false;
  flag3 = false;

  // Configuração dos periféricos
  act.servo_test(10.0, -10.0, 2.0);
  att_est.init();
  ver_est.init();

  // Definição da taxa de amostragem
  amostragem.attach(&callback_main, dt);

  input.attach(&callback_ref, dt_wave);

  range_sample.attach(&callback_range, dt_range);


  // Arm the sistem to initialization
  init.arm();

  while (true) {
    if (flag1) {

      flag1 = false;

      // Input references
      ref_gen.ref_angle(theta_ref, phi_ref);
      ref_gen.ref_vertical(vertical_ref);


      // Control and OBS step
      att_est.read();
      if (flag3) {
        flag3 = false;
        ver_est.read(att_est.Phi, att_est.Theta);
      }

      att_est.estimate(att_cont.f_x, att_cont.f_y, att_est.Theta, att_est.Q, att_est.Phi, att_est.P);
      ver_est.estimate(ver_cont.f_z, ver_est.z);
      
      att_cont.control(ref_gen.u_ref_phi, ref_gen.u_ref_theta, ref_gen.ref_phi, ref_gen.ref_theta, att_est.estimated_phi, att_est.estimated_theta);
      ver_cont.control(ref_gen.u_ref_z, ref_gen.ref_z, ver_est.estimated_z);

      // Actuate signals
      act.actuate_servos(att_cont.f_x, att_cont.f_y, ver_cont.f_z);
      act.actuate_valve(att_cont.f_x, att_cont.f_y, ver_cont.f_z);

      // Print my states
    //   pc.printf("STATE1= %f, STATE2= %f, STATE3=%f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2]);
    //   pc.printf("%f %f %f %f %f %f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2], att_cont.f_x, obs.Theta, obs.Q);
    //   pc.printf("%f %f %f\n\r", (180.0 * obs.estimated[0]  / pi), (180.0 * obs.estimated[1] / pi), (180.0 * obs.estimated[2] / pi));
    //   pc.printf("%f %f %f %f\n\r", att_cont.f_x, obs.estimated[0], obs.estimated[1], obs.estimated[2]);
    //   bt.printf("FX= %f, THETA= %f\n\r", att_cont.f_x, (180.0 * obs.estimated_theta[0] / pi));
      bt.printf("z=%f, z_hat= %f, z_dot_hat= %f, f_z= %f\n\r", ver_est.z, ver_est.estimated_z[0], ver_est.estimated_z[1],ver_cont.f_z);

      
    //   bt.printf("%f %f %f %f %f %f\n\r", att_cont.f_x, (180.0 * obs.Theta / pi), (180.0 * obs.estimated_theta[0] / pi), (180.0 * obs.Q / pi), (180.0 * obs.estimated_theta[1] / pi), (180.0 * obs.estimated_theta[2] / pi));
    //   pc.printf("Q= %f, Q_hat= %f\n\r", (180.0 * obs.Q / pi), (180.0 * obs.estimated[1] / pi));
    //   pc.printf("%f %f\n\r", act.theta, (180.0 * obs.estimated_log[0] / pi));
        //  bt.printf("%f %f\n\r", obs.Theta, obs.Q);
    //   pc.printf("FX= %f\n\r", att_cont.f_x);
    //   pc.printf("%f %f %f %f %f %f\n\r", obs.estimated[0], obs.estimated[1], obs.estimated[2], att_cont.f_x, obs.Theta, obs.Q);
    //   bt.printf("%f %f\n\r", att_cont.f_x, (180.0 * obs.estimated_theta[0] / pi));
    
      

      wave_input();
    }

    
  }
}

void callback_main() { flag1 = true; }

void callback_ref() { flag2 = !flag2; }

void callback_range() { flag3 = true; }


void wave_input() {
  if (flag2 == true) {
    theta_ref = 0.0;//(pi * 30) / 180;
    phi_ref = 0.0;//-(pi * 30) / 180;
  }
  if (flag2 == false) {
    theta_ref = 0.0;//-(pi * 10)/180;
    phi_ref = 0.0;//(pi * 30) / 180;;
  }
}
