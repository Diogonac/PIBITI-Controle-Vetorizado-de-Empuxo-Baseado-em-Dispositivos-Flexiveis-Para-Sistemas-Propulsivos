#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX

Calibration calib;


int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  calib.config_dac();
  wait(10);

  while (calib.init_dac == true) {

    calib.calib_dac();
    wait(5);
   }
  }
