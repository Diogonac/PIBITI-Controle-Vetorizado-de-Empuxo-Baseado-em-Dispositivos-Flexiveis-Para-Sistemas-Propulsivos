#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Imports
EstimadorAtitude att_est;
VerticalEstimator ver_est;

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
  att_est.init();
  ver_est.init();

  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_range);

  while (true) {
    if (flag) {
      flag = false;
      att_est.estimate();
      ver_est.predict(0.0);
      if (flag_range) {
        flag_range = false;
        ver_est.correct(att_est.Phi, att_est.Theta);
        //ver_est.correct(0,0);

        pc.printf("z [m]:%6.2f | w [m/s]:%6.2f | Phi[°0]:%.2f | Theta[°0]:%.2f \n", ver_est.z, ver_est.w, att_est.Phi, att_est.Theta);
      }
    }
  }
}