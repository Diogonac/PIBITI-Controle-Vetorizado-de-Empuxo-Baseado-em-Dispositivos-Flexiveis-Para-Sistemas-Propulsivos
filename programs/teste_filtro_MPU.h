#include "imports.h"
#include "mbed.h"

//============ Configura porta serial ===============
Serial pc(SERIAL_TX1, SERIAL_RX1); // Comunicação com USB TX, RX
// Define serial command variable
char command;

// Imports
Mixer mixer;
ControladorAtitude cont_atitude;
EstimadorAtitude estima_atitude;
InicializaPerifericos inicializa;

// Ticker para taxa de amostragem
Ticker amostragem;

Timer tempo;

// Flag de segurança
bool flag;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r;

void callback(void);

int main() {
  tempo.start();

  pc.baud(9600); // Define a velocidade da porta USB

  // Defino o valor das veriáveis de referência
  phi_r = 0.0;
  theta_r = 0.0;
  p_r = 0.0;
  q_r = 0.0;

  // Configuração dos periféricos
  mixer.config_servos();
  estima_atitude.config_imu();
  mixer.config_MPU();

  inicializa.verifica(mixer.verifica_servos, estima_atitude.verifica_imu,
                      mixer.verifica_MPU);

  // Definição da taxa de amostragem
  amostragem.attach(&callback, dt);

  // mixer.calibra_servo_MPU();

  while (inicializa.sistema_verificado) {

    if (flag) {

      flag = false;

      mixer.estima_MPU();
    //   pc.printf("phi= %f, theta= %f,psi= %f\n", mixer.Phi_MPU, mixer.Theta_MPU, mixer.Psi_MPU);
    }

    //Print attitude
    if (pc.readable()) {
      command = pc.getc();
      if (command == 'p') {
        pc.printf("%f,%f,%f\n", mixer.Phi_MPU, mixer.Theta_MPU, mixer.Psi_MPU);
      }
    }
  }
}

void callback() { flag = true; }