#include "imports.h"
#include "mbed.h"
#include <cstdio>


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX

// Imports
Actuators act;
Calibration calib;
Initialization init;

int main() {

  pc.baud(115200); // Define a velocidade da porta USB
  calib.config_calib_imu();
  wait(2);
  act.servo_test(110.0, 70.0, 2.0);
  wait(2);
  init.arm();

  // while (calib.calibration != true) {
  while (true) {
    calib.test_calib();

    //  calib.calibra_servo_phi();
    //  wait(1);
    //  calib.calibra_servo_theta();
    //  wait(1);
  }

  act.safe_state();
}