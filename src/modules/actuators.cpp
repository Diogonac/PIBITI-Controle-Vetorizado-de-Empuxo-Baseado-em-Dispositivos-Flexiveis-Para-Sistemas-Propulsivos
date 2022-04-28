#include "actuators.h"
#include <cstdio>

// Classe do construtor
Actuators::Actuators()
    : valve(SDA, SCL), LED_amarelo(AMARELO), servo1(SERVO1), servo2(SERVO2) {

  LED_amarelo = 0;

  phi_servo1 = 0.0;
  theta_servo2 = 0.0;

  phi = 0.0;
  theta = 0.0;

  pos = 0.0;
  time = 0.0;

  voltageValve = 1.0;

  init_servos = false;
  init_dac = false;
}

void Actuators::config_dac() {

  if (valve.open()) {
    printf("Device detected!\n");
    valve.wakeup();
    init_dac = true;
  } else {
    printf("Device not detected!\n");
  }
}

double Actuators::safe_angle(double angulo) {

  if (angulo > 100.0 & angulo < 70.0) {

    angulo = 90.0;
  } else {
    angulo = angulo;
  }
  return angulo;
}

void Actuators::safe_state(void) {

  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 1;

  // Aciona os servos
  servo1.position(phi_safe);
  servo2.position(phi_safe);

  wait(1);
}

/* Aciona a válvula para entregar o empuxo (N) total desejado (baseado nos
vetores empuxo) e os servos (graus) */
void Actuators::actuate_servos(double f_x, double f_y, double f_z) {
  // Contição dos avisos sonoros e luminosos
  LED_amarelo = 0;

  // Calcula os ângulos desejados do ponto central
  calc_thruster(f_x, f_y, f_z);

  // Calcula o ângulo atual dos servos
  phi = PHI(phi_servo1 + offset_servo1);
  theta = THETA(theta_servo2 + offset_servo2);

  // Aciona os servos
  servo1.position(safe_angle(phi));
  servo2.position(safe_angle(theta));
}

/* Aciona a válvula para entregar o empuxo (N) total desejado */
void Actuators::actuate_valve(double f_x, double f_y, double f_z) {

  calc_thruster(f_x, f_y, f_z);

  // DC voltage on valve to achieve the amount of thrust
  //    for (float i = 0.0; i < 360.0; i += 0.1){
  //       valve.write(0.5 * (sinf(i * 3.14159265 / 180.0) + 1));
  //    }

  if (total_thruster <= 6.105) {
    voltageValve = coef1 * pow(total_thruster, 3) + coef2 * pow(total_thruster, 2) + coef3 * total_thruster + coef4;
  } else{
    voltageValve = coef5 * total_thruster + coef6;
  }

  valve.write(voltageValve);
   //valve.write(0.0);
}

/* Converte os vetores de empuxo no vetor empuxo total para calcular a abertura
da válvula e os ângulos desejados nos tamanhos de pulsos */
void Actuators::calc_thruster(double f_x, double f_y, double f_z) {

  // Calcula o empuxo total
  total_thruster = sqrt((f_x * f_x) + (f_y * f_y) + (f_z * f_z));

  // Calcula os ângulos | phi --> roll(x) / theta --> pitch(y)
  phi_servo1 = ((180.000f * atan2(-f_y, f_z)) / pi);
  theta_servo2 = ((180.000f * atan2(f_x, f_z)) / pi);
}

void Actuators::servo_test(double max_angle, double min_angle,
                           double step_angle) {

  safe_state();
  wait(1);
  LED_amarelo = 0;

  printf("\r\n");                    // Pula uma linha no leitor
  printf("Inicio da varredura\r\n"); // Indica que a varredura de 30º nos dois
                                     // servos foi iniciada

  time = 100; // time_displacement*step_angle;

  for (pos = 90.0; pos <= max_angle; pos += step_angle) {
    // Varre a abertura de 90º até 105º com um incremento de 0.15º
    servo1.position(PHI(pos));   // Imprime o Ângulo no servo 1
    servo2.position(THETA(pos)); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(time); // Aguarda o deslocamento
  }

  for (pos = max_angle; pos >= min_angle; pos -= step_angle) {
    // Varre a abertura de 105º até 75º com um incremento de 0.15º
    servo1.position(PHI(pos));   // Imprime o Ângulo no servo 1
    servo2.position(THETA(pos)); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(time); // Aguarda o deslocamento
  }

  for (pos = min_angle; pos <= 90.0; pos += step_angle) {
    // Varre a abertura de 75º até 90º com um incremento de 0.15º
    servo1.position(PHI(pos));   // Imprime o Ângulo no servo 1
    servo2.position(THETA(pos)); // Imprime o Ângulo no servo 2
    printf("Posição: %f \r\n", pos);
    wait_ms(time); // Aguarda o deslocamento
  }

  wait(2);
  printf("Varredura completa\r\n");

  init_servos = true;
}

double Actuators::PHI(double phi_angle) { return (P1 * phi_angle) + P2; }

double Actuators::THETA(double theta_angle) { return (T1 * theta_angle) + T2; }