#include "imports.h"
#include "mbed.h"


//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1, 115200); // Comunicação com USB TX, RX

// Imports
EstimadorAtitude att_est;
Actuators act;

// Ticker para taxa de amostragem
Ticker amostragem;

// Flag de segurança
bool flag;

void callback(void);

int main() {

  pc.baud(115200); // Define a velocidade da porta USB

  att_est.init();
  act.config_dac();

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

  while (act.init_dac == true) {
    if (flag) {

      flag = false;

      att_est.estimate();

      printf("Phi= %f, Theta= %f, Psi= %f ", (180.0 * att_est.Phi / pi),
             (180.0 * att_est.Theta / pi), (180.0 * att_est.Psi / pi));
      printf("GX= %f, GY= %f, GZ= %f \r\n", att_est.P, att_est.Q, att_est.R);

      act.actuate_valve(0, 0, 0);
    }
  }
}

void callback() { flag = true; }