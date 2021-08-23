#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

//================ Configura servos =================
Servo servo1(D3);
Servo servo2(SERVO2);

int count_varredura = 0; //Contador par indicar a quantidade de varreduras efetuadas
double pos = 0; //Contador para armazenar o angulo atual dos servos

//============ Declara as funções utilizadas ==========
void configuracao_servos(void);
void varredura_servos(float tempo);

int main()
{

    pc.baud(115200); //Define a velocidade da porta USB

    configuracao_servos();

    while(1) {

        varredura_servos(0.5); //0.105
        wait(10);

    }
}

void configuracao_servos()
{

//=================== Calibração dos servos =====================
    servo1.calibrate(0.0019, 0.001, 180.0); //Define as configurações do servo 1: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total
    servo2.calibrate(0.0019, 0.001, 180.0); //Define as configurações do servo 2: pulsewidth_MÁX / pulsewidth_MíN / Ângulo de varredura total

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
    for (pos = 60.0; pos <= 120.0; pos += 0.005) {
        //Varre a abertura de 90º até 105º com um incremento de 1º
        //servo1.position(pos); //Imprime o Ângulo no servo 1
        servo2.position(pos); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Angulo atual %f\r\n", pos); //Indica o ângulo atual dos servos

    }

    for (pos = 120.0; pos >= 60.0; pos -= 0.5) {
        //Varre a abertura de 105º até 75º com um incremento de 1º
        //servo1.position(pos); //Imprime o Ângulo no servo 1
        servo2.position(pos); //Imprime o Ângulo no servo 2
        wait_ms(tempo); //Aguarda o deslocamento
        pc.printf("Angulo atual %f\r\n", pos); //Indica o ângulo atual dos servos
    }
    

    count_varredura++;

    pc.printf("Varredura %d completa\r\n", count_varredura); //Indica que a varredura de 30º nos dois servos foi concluida

}