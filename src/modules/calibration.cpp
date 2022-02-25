#include "calibration.h"
#include <cstdio>

EstimadorAtitude estima_atitude;

// Classe do construtor
Calibration::Calibration() : servo1(SERVO1), servo2(SERVO2){

  calibration = false;
  calibration_phi = false;
  calibration_theta = false;
  estima_atitude.config_imu();

  sum_phi = 0.0;
  sum_theta = 0.0;

  estimated_phi = 0.0;
  estimated_theta = 0.0;

  mean_phi = 0.0;
  mean_theta = 0.0;
  
}

void Calibration:: check_calibration (bool calib_phi, bool calib_theta){

    if(calib_phi == true && calib_theta == true){

        calibration = true;
    }

    else{
        calibration = false;
    }

}

void Calibration::calibra_servo_phi(void) {

  printf("\r\n");
  printf("Calibração servo 1 ângulo phi iniciada");
  printf("\r\n");
  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);
  estima_atitude.estimate();
  wait(1);


  for (int i = 0; i < 6; i++) {
    servo1.position(lista_angulos_1[i] + offset_servo1);
    servo2.position(offset_servo2);
    wait(2);
    sum_phi = 0.0;

    for (int y = 0; y < 500; y++) {
      estima_atitude.estimate();
      estimated_phi = (estima_atitude.Phi * 180.0 / pi);
      sum_phi += estimated_phi;
      wait_ms(2);

      //printf("%f %f\r\n", estimated_phi, lista_angulos_1[i] + offset_servo1);
    }

    mean_phi = sum_phi / 500;
    phi_data_calib[i] = mean_phi + offset_servo1; 
    printf("%f %f\r\n", mean_phi + offset_servo1, lista_angulos_1[i] + offset_servo1);
    wait(2);
  }

    servo1.position(offset_servo1);
    servo2.position(offset_servo2);
    wait(1);

    for (int i = 0; i < 6; i++) {
    servo1.position(lista_angulos_2[i] + offset_servo1);
    servo2.position(offset_servo2);
    wait(2);
    sum_phi = 0.0;

    for (int y = 0; y < 500; y++) {
      estima_atitude.estimate();
      estimated_phi = (estima_atitude.Phi * 180.0 / pi);
      sum_phi += estimated_phi;
      wait_ms(2);

      //printf("%f %f\r\n", estimated_phi, lista_angulos_2[i] + offset_servo1);
    }

    mean_phi = sum_phi / 500;
    phi_data_calib[i] = mean_phi + offset_servo1; 
    printf("%f %f\r\n", mean_phi + offset_servo1, lista_angulos_2[i] + offset_servo1);
    wait(2);
  }

  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);

  printf("Calibração servo 1 ângulo phi finalizada");
  printf("Resultados\r\n");

  printf("Angulos_REF= [");
  for(int k = 0; k < 11; k++){
    printf(", %f", lista_angulos[k]);
  }
  printf(" ];");

  printf("\r\n");
  printf("Angulos_Phi= [");
  for(int k = 0; k < 11; k++){
    printf(", %f", phi_data_calib[k]);
  }
  printf(" ];");

  calibration_phi = true;

}

void Calibration::calibra_servo_theta(void) {

  printf("\r\n");
  printf("Calibração servo 1 ângulo theta iniciada");
  printf("\r\n");
  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);
  estima_atitude.estimate();
  wait(1);


  for (int i = 0; i < 6; i++) {
    servo1.position(offset_servo1);
    servo2.position(lista_angulos_2[i] + offset_servo2);
    wait(2);
    sum_theta = 0.0;

    for (int y = 0; y < 500; y++) {
      estima_atitude.estimate();
      estimated_theta = (estima_atitude.Theta * 180.0 / pi);
      sum_theta += estimated_theta;
      wait_ms(2);

      //printf("%f %f\r\n", estimated_theta, lista_angulos_1[i] + offset_servo2);
    }

    mean_theta = sum_theta / 500;
    theta_data_calib[i] = mean_theta + offset_servo2; 
    printf("%f %f\r\n", mean_theta + offset_servo2, lista_angulos_1[i] + offset_servo2);
    wait(2);
  }

    servo1.position(offset_servo1);
    servo2.position(offset_servo2);
    wait(1);

    for (int i = 0; i < 6; i++) {
    servo1.position(offset_servo1);
    servo2.position(lista_angulos_2[i] + offset_servo2);
    wait(2);
    sum_theta = 0.0;

    for (int y = 0; y < 500; y++) {
      estima_atitude.estimate();
      estimated_theta = (estima_atitude.Theta * 180.0 / pi);
      sum_theta += estimated_theta;
      wait_ms(2);

      //printf("%f %f\r\n", estimated_theta, lista_angulos_2[i] + offset_servo2);
    }

    mean_theta = sum_theta / 500;
    theta_data_calib[i] = mean_theta + offset_servo2; 
    printf("%f %f\r\n", mean_theta + offset_servo2, lista_angulos_2[i] + offset_servo2);
    wait(2);
  }

  servo1.position(offset_servo1);
  servo2.position(offset_servo2);
  wait(1);

  printf("Calibração servo 1 ângulo theta finalizada");
  printf("Resultados\r\n");

  printf("Angulos_REF= [");
  for(int k = 0; k < 11; k++){
    printf(", %f", lista_angulos[k]);
  }
  printf(" ];");

  printf("\r\n");
  printf("Angulos_Theta= [");
  for(int k = 0; k < 11; k++){
    printf(", %f", theta_data_calib[k]);
  }
  printf(" ];");

  calibration_theta = true;

}

void Calibration::phi_test_calib(void){


}