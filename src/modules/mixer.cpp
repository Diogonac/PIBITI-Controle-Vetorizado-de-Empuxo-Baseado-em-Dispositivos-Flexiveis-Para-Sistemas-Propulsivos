#include "mixer.h"
#include <cstdio>

// EstimadorAtitude estima_atitude;

// Classe do construtor
Mixer::Mixer() : valvula(VALVULA), LED_amarelo(AMARELO), servo1(SERVO1), servo2(SERVO2), MPU6050(MPU_SDA, MPU_SCL) {

  LED_amarelo = 0;
  //
  phi_total = offset_servo1;
  theta_total = offset_servo2;

  verifica_servos = false;

  pos = 0.0;

  delta_angulos[0] = offset_servo1;
  delta_angulos[1] = offset_servo2;

  tempo_servos = 5.0;

    Phi_MPU = 0.0;
    Theta_MPU = 0.0;
    Psi_MPU = 0.0;

}

void Mixer::config_servos(void) {

  //=================== Calibração dos servos =====================
  servo1.calibrate(2500, 550, 180.0); // Define as configurações do servo 1: pulsewidth_MÁX
                           // / pulsewidth_MíN / Ângulo de varredura total
  servo2.calibrate(2500, 550, 180.0); // Define as configurações do servo 2: pulsewidth_MÁX
                           // / pulsewidth_MíN / Ângulo de varredura total

  //=================== Verificação dos servos ====================
  servo1.position(offset_servo1); // Define a posição inicial do servo 1
  servo2.position(offset_servo2); // Define a posição inicial do servo 2
  wait_ms(10);                    // Aguarda o deslocamento

  printf("\r\n");                    // Pula uma linha no leitor
  printf("Inicio da varredura\r\n"); // Indica que a varredura de 30º nos dois
                                     // servos foi iniciada

  for (pos = 90; pos <= 105; pos += 0.15) {
    // Varre a abertura de 90º até 105º com um incremento de 1º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  for (pos = 105; pos >= 75; pos -= 0.15) {
    // Varre a abertura de 105º até 75º com um incremento de 1º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 27
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  for (pos = 75; pos <= 90; pos += 0.15) {
    // Varre a abertura de 75º até 90º com um incremento de 1º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  wait(2);
  printf("Varredura completa\r\n"); // Indica que a varredura de 30º nos dois
                                    // servos foi concluida

  servo1.position(offset_servo1); // Define a posição inicial do servo 1
  servo2.position(offset_servo2); // Define a posição inicial do servo 2

  verifica_servos = true;
}

void Mixer::estado_seguro(void) {

  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 1;

  servo1.position(offset_servo1);
  servo2.position(offset_servo2);

  wait_ms(tempo_servos);
}

/* Aciona a válvula para entregar o empuxo (N) total desejado (baseado nos
vetores empuxo) e os servos (graus) */
void Mixer::actuate(double f_x, double f_y, double f_z) {
  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 0;

  mixer(f_x, f_y, f_z);

  // Calcula o deslocamento necessário
  desloca_phi = rint((((180.000f * phi_servo1) / pi)) * 1000.0) / 1000.0;
  desloca_theta = rint((((180.000f * theta_servo2) / pi)) * 1000.0) / 1000.0;

  // Calcula o ângulo que o os servos devem se mover
  phi_total = (desloca_phi + offset_servo1); 
  theta_total = (desloca_theta + offset_servo2);

  // Calcula o delta angulo que será impresso nos servos
  delta_phi = abs(phi_total - delta_angulos[0]);
  delta_theta = abs(theta_total - delta_angulos[1]);

  if (phi_total > theta_total) {

    tempo_servos = delta_phi * constante_velocidade;

  } else {

    tempo_servos = delta_theta * constante_velocidade;
  }

  if (tempo_servos < 5.0) {

    tempo_servos = 5.0;
  }

  if (tempo_servos > 20.0) {

    tempo_servos = 20.0;
  }

  // Aciona os servos
  servo1.position(phi_total);
  servo2.position(theta_total);
  wait_ms(tempo_servos);

  delta_angulos[0] = phi_total;
  delta_angulos[1] = theta_total;

  // Aciona a valvula
  valvula = controle_valvula(empuxo_total);

}

void Mixer::calibra_servo_MPU(void) {

  for (int i = 0; i < 16; i++) {

    servo1.position(offset_servo1 + lista_angulos[i]);
    // servo2.position(offset_servo2 + lista_angulos[i]);
    wait(2);
    estima_MPU();
    printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 + Theta_MPU),(offset_servo1 + lista_angulos[i]));
    wait(2);
  }

  servo1.position(offset_servo1);
  wait(1);

    for (int i = 0; i < 16; i++) {

    servo1.position(offset_servo1 - lista_angulos[i]);
    // servo2.position(offset_servo2 + lista_angulos[i]);
    wait(2);
    estima_MPU();
    printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 - Theta_MPU),(offset_servo1 - lista_angulos[i]));
    wait(2);
  }

  servo1.position(offset_servo1);
  wait(1);

  for (int i = 0; i < 16; i++) {

    // servo1.position(offset_servo1 + lista_angulos[i]);
    servo2.position(offset_servo2 + lista_angulos[i]);

    wait(2);
    estima_MPU();
    printf("Phi_MPU= %f, Servo= %f \r\n",(90.0 + Phi_MPU),(offset_servo2 + lista_angulos[i]));
    wait(2);
  }

  servo2.position(offset_servo2);
  wait(1);

    for (int i = 0; i < 16; i++) {

    // servo1.position(offset_servo1 + lista_angulos[i]);
    servo2.position(offset_servo2 - lista_angulos[i]);

    wait(2);
    estima_MPU();
    printf("Phi_MPU= %f, Servo= %f \r\n",(90.0 - Phi_MPU),(offset_servo2 - lista_angulos[i]));
    wait(2);
  }

  servo2.position(offset_servo2);
  wait(1);

}

/* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
da válvula e os ângulos desejados nos tamanhos de pulsos */
void Mixer::mixer(double f_x, double f_y, double f_z) {

  // Calcula o empuxo total
  empuxo_total = sqrt((f_x * f_x) + (f_y * f_y) + (f_z * f_z));

  // Calcula os ângulos | phi --> roll(x) / theta --> pitch(y)
  phi_servo1 = -1 * atan((-1 * f_y) / f_z);
  theta_servo2 = atan(f_x / f_z);
}

// Converte o empuxo total em sinal para abertura da válvula
double Mixer::controle_valvula(double abertura_valvula) {

  return abertura_valvula;
}


// Inicializa a IMU
void Mixer::config_MPU()
{

    printf("Iniciando MPU6050\r\n");

    if(MPU6050.testConnection() == 1) {

        printf("MPU6050 - Status: OK\r\n");

    } else {

        printf("MPU6050 - Status: Sem resposta \r\n");

    }

    MPU6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    MPU6050.setGyroRange(MPU6050_GYRO_RANGE_250);
    MPU6050.setBW(MPU6050_BW_20);

    wait_ms(10); //Aguarda o deslocamento

}

// Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
void Mixer::estima_MPU()
{

    MPU6050.getAccelero(acc_MPU);

    // Pitch e Roll estão trocados conforme a convensão utilizada
    ax = acc_MPU[0];
    ay = acc_MPU[1];
    az = acc_MPU[2];

    Phi_MPU = atan2((ay),(az)) * 180 / pi;
    Theta_MPU = atan2((ax),(az)) * 180 / pi;
}