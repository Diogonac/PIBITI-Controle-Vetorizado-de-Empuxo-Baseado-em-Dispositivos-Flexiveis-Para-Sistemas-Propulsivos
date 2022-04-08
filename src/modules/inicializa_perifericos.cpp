#include "inicializa_perifericos.h"

// Classe do construtor
InicializaPerifericos::InicializaPerifericos():LED_verde(VERDE), LED_vermelho(VERMELHO), LED_amarelo(AMARELO),buzzer(BUZZER)
{

    LED_verde = 0;
    LED_vermelho = 0;
    LED_amarelo = 0;

    sistema_verificado = 0;


}

void InicializaPerifericos::verifica(bool estado_inicializacao_servos, bool estado_inicializacao_imu, bool estado_inicializacao_dac)
{

    if(estado_inicializacao_servos == 1 && estado_inicializacao_imu == 1 && estado_inicializacao_dac == 1) { //&& verifica_imu == true
    
        aviso_sonoro();
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

void InicializaPerifericos::aviso_sonoro(void){
    
        buzzer.write(0.7);
        wait_ms(200);
        buzzer.write(0);
        wait_ms(200);
        buzzer.write(0.7);
        wait_ms(200);
        buzzer.write(0);
        
    
    }