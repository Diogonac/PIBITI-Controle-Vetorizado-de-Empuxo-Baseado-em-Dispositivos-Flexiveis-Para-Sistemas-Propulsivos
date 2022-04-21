#include "imports.h"
#include "mbed.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Imports
VerticalEstimator ver_est;
Actuators act;
Initialization init;
VerticalController ver_cont;


// Define ticker
Ticker tic, tic_range;

// Define interrupt flags
bool flag, flag_range;

// Define callback functions
void callback() { flag = true; }
void callback_range() { flag_range = true; }

int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  // Set references
  float z_r = 0.2;

  // Initialize estimator objects
  ver_est.init(); 
  act.config_dac();

  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_range);

  // Arm the sistem to initialization 
  init.arm();

    while (init.valve_flag == 1){
    if (flag) {
      flag = false;
      ver_est.predict(m*g);
      act.actuate_valve(0, 0, 0);

      if (flag_range) {
        flag_range = false;
        //ver_est.correct(att_est.Phi, att_est.Theta);
        ver_est.correct(0.0,0.0);
      }

      ver_cont.control(z_r, ver_est.z, ver_est.w);
      ver_cont.feed_foward(ver_cont.f_z, ver_est.w, f_max, K1, K2, K3, f_stiction, f_coulomb, z_r, ver_est.z);
      pc.printf("%0.2f %0.2f %0.2f %0.2f\n", ver_cont.uff, ver_cont.f_z, ver_cont.uout, ver_est.w);

    }
  }
}