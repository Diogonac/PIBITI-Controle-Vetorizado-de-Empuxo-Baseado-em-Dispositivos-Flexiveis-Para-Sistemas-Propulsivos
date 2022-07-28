#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação com USB TX, RX

// Imports
AttitudeObserver att_obs;
VerticalObserver ver_est;

// Define ticker
Ticker tic, tic_range;

// Define interrupt flags
bool flag, flag_range;

// Define callback functions
void callback() { flag = true; }
void callback_range() { flag_range = true; }

// Main program
int main() {
  // Initialize estimator objects
  att_obs.initIMU();
  ver_est.init();

  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_range);

  while (true) {
    if (flag) {
      flag = false;
      att_obs.readIMU();
      ver_est.predict(0.0);
      if (flag_range) {
        flag_range = false;
        ver_est.correct(att_obs.Phi, att_obs.Theta);
        //ver_est.correct(0,0);

        bt.printf("z [m]:%6.2f | w [m/s]:%6.2f | Phi[°0]:%.2f | Theta[°0]:%.2f \n", ver_est.z, ver_est.w, att_obs.Phi, att_obs.Theta);
      }
    }
  }
}