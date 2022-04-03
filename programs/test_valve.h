#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX

Actuators act;

int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  act.config_dac();
  wait(1);

  while (act.init_dac == true) {

   act.actuate_valve(0, 0, 0);

  }
}