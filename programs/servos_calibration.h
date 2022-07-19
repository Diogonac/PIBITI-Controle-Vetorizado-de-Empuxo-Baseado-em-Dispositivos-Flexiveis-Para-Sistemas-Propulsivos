#include "imports.h"
#include "mbed.h"
#include <cstdio>

//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX

// Imports
Actuators act;
Calibration calib;
Initialization init;

bool flag, run;
double ref_theta;
void wave_input(void);
void callback(void);

Ticker input;
Ticker sample;

int main() {

  pc.baud(115200); // Define a velocidade da porta USB
  calib.config_calib_imu();
  wait(2);
  act.servo_test(10.0, -10.0, 2.0);
  wait(2);
  init.arm();
  flag = false;
  ref_theta = 0.0;
  input.attach(&wave_input, dt_wave);
  sample.attach(&callback, dt);

  // while (calib.calibration != true) {
  while (true) {

    if (run) {

       run = false;
      calib.test_calib();

    //    calib.calibra_servo_phi();
    //    wait(1);
    //    calib.calibra_servo_theta();
    //    wait(1);

    //   calib.servo_input_waves(ref_theta, 1.0);

    //   if (flag == true) {
    //     ref_theta = 10;
    //   }
    //   if (flag == false) {
    //     ref_theta = 0.0;
    //   }
    }
  }

  act.safe_state();
}

void wave_input() { flag = !flag; }
void callback() { run = true; }