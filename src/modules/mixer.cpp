#include "mixer.h"
#include <cstdio>

// EstimadorAtitude estima_atitude;

// Classe do construtor
Mixer::Mixer() : valvula(VALVULA), LED_amarelo(AMARELO), servo1(SERVO1), servo2(SERVO2), MPU6050(MPU_SDA, MPU_SCL) {

  LED_amarelo = 0;

  phi_total = offset_servo1;
  theta_total = offset_servo2;

  verifica_servos = false;
  verifica_MPU = false;

  pos = 0.0;

  delta_angulos[0] = offset_servo1;
  delta_angulos[1] = offset_servo2;

  tempo_servos = 5.0;

  // Condição inicial dos angulos de Euler com o filtro complementar
  Phi_MPU = 0.0;
  Theta_MPU = 0.0;
  Psi_MPU = 0.0;

  // Condição inicial dos angulos de Euler pelos dados do acc
  Phi_MPU_a = 0.0;
  Theta_MPU_a = 0.0;
  
  // Condição inicial dos angulos de Euler pelos dados do gyr
  Phi_MPU_g = 0.0;
  Theta_MPU_g = 0.0;
  Psi_MPU_g = 0.0;

  // Condição inicial das médias e somatórias dos angulos para a calibração dos servos
  Phi_MPU_MM = 0.0;
  Theta_MPU_MM = 0.0;

  sum_phi = 0.0;
  sum_theta = 0.0;

  // Condição inicial do erro sistemático para correção na integração numérica
  p_bias = 0.0;
  q_bias = 0.0;
  r_bias = 0.0;
  
}

double Mixer::seguranca_servos(double angulo) {

  if (angulo > 105.0 | angulo < 75.0) {

    angulo = 90.0;
  } else {
    angulo = angulo;
  }
  return angulo;
}

void Mixer::config_servos(void) {

  //=================== Calibração dos servos =====================
  /* Define as configurações dos servos: pulsewidth_MÁX pulsewidth_MíN / Ângulo de varredura total */
  servo1.calibrate(2500, 550, 180.0); 
  servo2.calibrate(2500, 550, 180.0); 

  //=================== Verificação dos servos ====================

  // Correção dos ângulos para a posição de segurança
  theta_calib = (offset_servo2 - t2) / t1; // Define a posição inicial do servo 2
  phi_calib = (offset_servo1 - p2) / p1; // Define a posição inicial do servo 1

  // Aciona os servos
  servo1.position(seguranca_servos(phi_calib));
  servo2.position(seguranca_servos(theta_calib));

  wait_ms(10); // Aguarda o deslocamento

  printf("\r\n");                    // Pula uma linha no leitor
  printf("Inicio da varredura\r\n"); // Indica que a varredura de 30º nos dois
                                     // servos foi iniciada

  for (pos = 90; pos <= 105; pos += 0.15) {
    // Varre a abertura de 90º até 105º com um incremento de 0.15º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  for (pos = 105; pos >= 75; pos -= 0.15) {
    // Varre a abertura de 105º até 75º com um incremento de 0.15º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 27
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  for (pos = 75; pos <= 90; pos += 0.15) {
    // Varre a abertura de 75º até 90º com um incremento de 0.15º
    servo1.position(pos); // Imprime o Ângulo no servo 1
    servo2.position(pos); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(5); // Aguarda o deslocamento
  }

  wait(2);
  printf("Varredura completa\r\n"); 

  // Correção dos ângulos para a posição de segurança
  theta_calib = (offset_servo2 - t2) / t1; // Define a posição inicial do servo 2
  phi_calib = (offset_servo1 - p2) / p1; // Define a posição inicial do servo 1

  // Aciona os servos
  servo1.position(seguranca_servos(phi_calib));
  servo2.position(seguranca_servos(theta_calib));

  wait_ms(10); // Aguarda o deslocamento

  verifica_servos = true;
}

void Mixer::estado_seguro(void) {

  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 1;

  // Correção dos ângulos para a posição de segurança
  theta_calib = (offset_servo2 - t2) / t1;
  phi_calib = (offset_servo1 - p2) / p1;

  // Aciona os servos
  servo1.position(seguranca_servos(phi_calib));
  servo2.position(seguranca_servos(theta_calib));

  wait_ms(10); // Aguarda o deslocamento
}

/* Aciona a válvula para entregar o empuxo (N) total desejado (baseado nos
vetores empuxo) e os servos (graus) */
void Mixer::actuate(double f_x, double f_y, double f_z) {
  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 0;

  // Calcula os ângulos desejados do ponto central
  mixer(f_x, f_y, f_z);

  // Converte os ângulos desejados do ponto central em grau)
  desloca_phi = rint((((180.000f * phi_servo1) / pi)) * 1000.0) / 1000.0;
  desloca_theta = rint((((180.000f * theta_servo2) / pi)) * 1000.0) / 1000.0;

  // Calcula o ângulo que o os servos devem se mover
  phi_total = (desloca_phi + offset_servo1);
  theta_total = (desloca_theta + offset_servo2);

  // Calcula o delta angulo que será impresso nos servos
  delta_phi = abs(phi_total - delta_angulos[0]);
  delta_theta = abs(theta_total - delta_angulos[1]);

  // Verifica a necessidade do dalay para o deslocamento completo dos servos
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

  // Calibra os ângulos do servo
  theta_calib = (theta_total - t2) / t1;
  phi_calib = (phi_total - p2) / p1;

  // Implementar rotina de verificação de angulo, fora do intervalo 75 - 105 ->
  // desconsiderar

  // Aciona os servos
  servo1.position(seguranca_servos(phi_calib));
  servo2.position(seguranca_servos(theta_calib));

  wait_ms(tempo_servos); // Aguarda o deslocamento

  estima_MPU(); // Coleta os dados do MPU

  printf("%f %f %f %f\r\n", Phi_MPU, Theta_MPU, desloca_phi, desloca_theta);

  // Armazena os dados do ângulo nos servos
  delta_angulos[0] = phi_calib;
  delta_angulos[1] = theta_calib;

  // Aciona a valvula
  valvula = controle_valvula(empuxo_total);
}

void Mixer::verifica_calib_servo_MPU(void) {

  wait(1);
  estima_MPU();
  wait(1);

  for (int i = 0; i < 31; i++) {
    // printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 + Theta_MPU),(offset_servo1
    // + lista_angulos[i]));
    printf("%f %f %f\r\n", Theta_MPU, lista_angulos[i], theta_calib);
    theta_calib = (lista_angulos[i] - t2) / t1;
    servo1.position(theta_calib);
    wait(2);
    estima_MPU();
  }

  servo1.position((90.0 - t2) / t1);
  wait(1);
  estima_MPU();
  wait(1);

  for (int i = 0; i < 31; i++) {
    // printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 + Theta_MPU),(offset_servo1
    // + lista_angulos[i]));
    printf("%f %f %f\r\n", Phi_MPU, lista_angulos[i], phi_calib);
    phi_calib = (lista_angulos[i] - p2) / p1;
    servo2.position(phi_calib);
    wait(2);
    estima_MPU();
  }

  servo2.position((90.0 - p2) / p1);
}

void Mixer::calibra_servo_MPU(void) {

  wait(1);
  estima_MPU();
  wait(1);

  for (int i = 17; i >= 0; i--) {
    // printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 + Theta_MPU),(offset_servo1
    // + lista_angulos[i]));
    theta_calib = (lista_angulos[i] - t2) / t1;
    servo2.position(seguranca_servos(lista_angulos[i] + 90.0));
    wait(2);
    sum_theta = 0.0;
    for (int y = 0; y < 499; y++) {
      estima_MPU();
      // printf("%f %f %f\r\n", -Theta_MPU, lista_angulos[i], theta_calib);
      sum_theta += -Theta_MPU;
      wait_ms(2);
    }
    Theta_MPU_MM = sum_theta / 500;
    printf("%f %f %f\r\n", Theta_MPU_MM, lista_angulos[i], theta_calib);
  }

  servo1.position(seguranca_servos(90.0));
  wait(1);

  for (int i = 17; i >= 0; i--) {
    // printf("Theta_MPU= %f, Servo= %f \r\n",(90.0 + Theta_MPU),(offset_servo1
    // + lista_angulos[i]));

    phi_calib = (lista_angulos[i] - p2) / p1;
    servo1.position(seguranca_servos(lista_angulos[i] + 90.0));
    wait(2);
    sum_phi = 0.0;
    for (int y = 0; y < 499; y++) {
      estima_MPU();
      // printf("%f %f %f\r\n", Phi_MPU, lista_angulos[i], phi_calib);
      sum_phi += Phi_MPU;
      wait_ms(2);
    }
    Phi_MPU_MM = sum_phi / 500;
    printf("%f %f %f\r\n", Phi_MPU_MM, lista_angulos[i], phi_calib);
  }

  servo2.position(seguranca_servos(90.0));
  wait(1);
}

/* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
da válvula e os ângulos desejados nos tamanhos de pulsos */
void Mixer::mixer(double f_x, double f_y, double f_z) {

  // Calcula o empuxo total
  empuxo_total = sqrt((f_x * f_x) + (f_y * f_y) + (f_z * f_z));

  // Calcula os ângulos | phi --> roll(x) / theta --> pitch(y)
  phi_servo1 = atan2(-f_y, f_z);
  theta_servo2 = atan2(f_x, f_z); // Verificar o uso de atan2
}

// Converte o empuxo total em sinal para abertura da válvula
double Mixer::controle_valvula(double abertura_valvula) {

  return abertura_valvula;
}

// Inicializa a IMU
void Mixer::config_MPU() {

  printf("Iniciando MPU6050\r\n");

  if (MPU6050.testConnection() == 1) {

    printf("MPU6050 - Status: OK\r\n");
    verifica_MPU = true;

  } else {

    printf("MPU6050 - Status: Sem resposta \r\n");
    verifica_MPU = false;
  }

  MPU6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_8G);
  MPU6050.setGyroRange(MPU6050_GYRO_RANGE_500);
  MPU6050.setBW(MPU6050_BW_256);

  wait_ms(10); 

  sum_p = 0.0;
  sum_q = 0.0;
  sum_r = 0.0;

  for (int i = 0; i < 500; i++) {

    // Leitura das velocidades angulares
    MPU6050.getGyro(gyr_MPU);

    gx = gyr_MPU[0];
    gy = gyr_MPU[1];
    gz = gyr_MPU[2];

    sum_p += gx;
    sum_q += gy;
    sum_r += gz;

    wait(dt);
  }

  p_bias = sum_p / 500;
  q_bias = sum_q / 500;
  r_bias = sum_r / 500; 

  printf("\r\n");
  printf("MPU6050 calibrado");

}

// Estima os ângulos de Euler (rad) e as velocidades angular (rad/s)
void Mixer::estima_MPU() {

  // Leitura das acelerações e velocidades angulares
  MPU6050.getAccelero(acc_MPU);
  MPU6050.getGyro(gyr_MPU);

  // Pitch e Roll estão trocados conforme a convensão utilizada
  ax = acc_MPU[0];
  ay = acc_MPU[1];
  az = acc_MPU[2];

  p = gyr_MPU[0] - p_bias;
  q = gyr_MPU[1] - q_bias;
  r = gyr_MPU[2] - r_bias;

  // Estima phi e theta com os dados do  acc
  Phi_MPU_a = (atan(ay / az) * 180.0 / pi); // ay e az não recebem - pois considera-se g+ (g rela e z imuestão em sentidos opostos)
  Theta_MPU_a = (atan2(ax, -(((az > 0) - (az < 0)) * sqrt(ay * ay + az * az))) * 180.0 / pi);

  // Estima phi, theta e psi com os dados do gyr
  Phi_MPU_g = Phi_MPU + (p + sin(Phi_MPU) * tan(Theta_MPU) * q + cos(Phi_MPU) * tan(Theta_MPU) * r) * dt;
  Theta_MPU_g = Theta_MPU + (cos(Phi_MPU) * q - sin(Phi_MPU) * r) * dt;
  Psi_MPU_g = Psi_MPU + (sin(Phi_MPU) * (1 / cos(Theta_MPU)) * q + cos(Phi_MPU) * (1 / cos(Theta_MPU)) * r) * dt;

  // Fusão dos dados do acc com o gyr para estimar os angulos e Euler com filtro complementar (LPF + HPF)
  Phi_MPU = (1 - alpha) * Phi_MPU_g + alpha * Phi_MPU_a; 
  Theta_MPU = (1 - alpha) * Theta_MPU_g + alpha * Theta_MPU_a; 
  Psi_MPU = Psi_MPU_g;

  // printf("%f,%f,%f\r\n", Phi_MPU, Theta_MPU, Psi_MPU);
}