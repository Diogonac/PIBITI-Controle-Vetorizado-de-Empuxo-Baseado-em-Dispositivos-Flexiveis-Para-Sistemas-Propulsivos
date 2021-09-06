#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

//================ Configura servos =================
Servo servo1(SERVO1);
Servo servo2(SERVO2);

int count_varredura = 0; //Contador par indicar a quantidade de varreduras efetuadas
double pos = 0; //Contador para armazenar o angulo atual dos servos

int pulso = 0;
int incremento_pulso = 0;


//============ Declara as funções utilizadas ==========
void configuracao_servos(void);
void varredura_servos(float tempo);
void varredura_servos_pulsos(float tempo);

int main()
{

    pc.baud(115200); //Define a velocidade da porta USB

    configuracao_servos();

    while(1) {

        //varredura_servos_pulsos(600); //0.105
        varredura_servos(50);
        //wait(5);

    }
}

void configuracao_servos()
{

//=================== Calibração dos servos =====================
    servo1.calibrate(2500, 550, 180.0); //Define as configurações do servo 1: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total
    servo2.calibrate(2500, 550, 180.0); //Define as configurações do servo 2: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total

//=================== Verificação dos servos ====================
    servo1.position(90.0); //Define a posição inicial do servo 1
    servo2.position(90.0); //Define a posição inicial do servo 2

    pc.printf("\r\n"); //Pula uma linha no leitor
    pc.printf("Inicio da varredura\r\n"); //Indica que a varredura de 30º nos dois servos foi iniciada
    wait_ms(5000); //Aguarda o deslocamento
    
}

void varredura_servos(float tempo)
{

//=================== Verificação dos servos ====================
    for (pos = 70.0; pos <= 110.0; pos += 2.0) {
        //Varre a abertura de 90º até 105º com um incremento de 1º
        servo1.position(pos); //Imprime o Ângulo no servo 1
        servo2.position(pos); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Angulo atual %f\r\n", pos); //Indica o ângulo atual dos servos

    }

    //wait(5);

    for (pos = 110.0; pos >= 70.0; pos -= 2.0) {
        //Varre a abertura de 105º até 75º com um incremento de 1º
        servo1.position(pos); //Imprime o Ângulo no servo 1
        servo2.position(pos); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Angulo atual %f\r\n", pos); //Indica o ângulo atual dos servos
    }
    

    count_varredura++;

    pc.printf("Varredura %d completa\r\n", count_varredura); //Indica que a varredura de 30º nos dois servos foi concluida

}


void varredura_servos_pulsos(float tempo)
{

//=================== Verificação dos servos ====================
    for (pulso = 550; pulso <= 2500; pulso += 50) {
        //Varre a abertura de 90º até 105º com um incremento de 1º
        servo1.position(pulso); //Imprime o Ângulo no servo 1
        servo2.largura_pulso(pulso); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Pulso atual %d\r\n", pulso); //Indica o ângulo atual dos servos

    }

    wait(5);

    for (pulso = 2500; pulso >= 550; pulso -= 50) {
        //Varre a abertura de 105º até 75º com um incremento de 1º
        servo1.position(pulso); //Imprime o Ângulo no servo 1
        servo2.largura_pulso(pulso); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Pulso atual %d\r\n", pulso); //Indica o ângulo atual dos servos
    }
    

    count_varredura++;

    pc.printf("Varredura %d completa\r\n", count_varredura); //Indica que a varredura de 30º nos dois servos foi concluida

}