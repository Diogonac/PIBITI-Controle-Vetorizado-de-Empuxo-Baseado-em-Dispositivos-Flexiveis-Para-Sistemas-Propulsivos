#include "initialization.h"

// Classe do construtor
Initialization::Initialization():LED_verde(VERDE), LED_vermelho(VERMELHO),buzzer(BUZZER)
{

    LED_verde = 0;
    LED_vermelho = 0;

    sistema_verificado = 0;
    valve_flag = 0;


}

void Initialization::verify(bool estado_inicializacao_servos, bool estado_inicializacao_imu, bool estado_inicializacao_dac)
{

    if(estado_inicializacao_servos == 1 && estado_inicializacao_imu == 1 && estado_inicializacao_dac == 1) { //&& verifica_imu == true
    
        arm();
        wait(1);

        LED_verde = 1;
        LED_vermelho = 0;
        sistema_verificado = 1;


    } else {

        buzzer.write(0.7);
        wait(1.5);
        buzzer.write(0);
        
        LED_verde = 0;
        LED_vermelho = 1;
        sistema_verificado = 0;
        
    }
}

void Initialization::arm(void){
    
        buzzer.write(0.0);
        LED_verde = 0;
        LED_vermelho = 1;
        wait_ms(500);
        buzzer.write(0.7);
        LED_verde = 0;
        LED_vermelho = 1;
        wait_ms(500);
        buzzer.write(0.0);
        LED_verde = 1;
        LED_vermelho = 1;
        wait_ms(500);
        buzzer.write(0.7);
        LED_verde = 1;
        LED_vermelho = 0;
        wait_ms(500);
        buzzer.write(0.0);

        valve_flag = 1;
    
    }