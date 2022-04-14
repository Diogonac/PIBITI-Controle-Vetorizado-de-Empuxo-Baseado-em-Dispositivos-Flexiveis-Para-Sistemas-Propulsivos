#include "mbed.h"
#include "imports.h"
#include <cstdio>

//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX

Actuators act;

double thrust[25] = {1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75, 6.0, 6.25, 6.5, 6.75, 7.0};
int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  act.config_dac();
  wait(10);

  while (act.init_dac == true) {
    
    for(int i=0; i<25; i++){
    act.actuate_valve(0, 0, thrust[i]);
    printf("Force: %f [N] | ValveV: %f [V]\r\n", thrust[i], act.voltageValve);
    wait(10);
   }
  }
}