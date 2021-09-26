#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

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


// Tempo de amostragem
const double dt = 0.02; // 50 Hz
void callback(void);

int main()
{
    tempo.start();
    
    pc.baud(115200); //Define a velocidade da porta USB

    // Defino o valor das veriáveis de referência
    phi_r = 0.0;
    theta_r = 0.0;
    p_r = 0.0;
    q_r = 0.0;

    // Configuração dos periféricos
    mixer.config_servos();
    estima_atitude.config_imu();
    mixer.config_MPU();
    
    inicializa.verifica(mixer.verifica_servos, estima_atitude.verifica_imu, mixer.verifica_MPU);
    

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    while(inicializa.sistema_verificado) {

        estima_atitude.estima();

        //p --> wx | q -- > wy | r --> wz
        while(abs(estima_atitude.Phi) <= pi /12.0 && abs(estima_atitude.Theta) <= pi /12.0 && abs(estima_atitude.P) <= 4.0* pi && abs (estima_atitude.Q) <= 4.0* pi && abs(estima_atitude.R) <= 4.0*pi) {

            if(flag) {
                
                flag = false;

                estima_atitude.estima();

                //pc.printf("YAW= %f, ROLL= %f, PITCH= %f, GIROX= %f, GIROY= %f \r\n", BNO055.euler.yaw, BNO055.euler.roll, BNO055.euler.pitch, BNO055.gyro.x, BNO055.gyro.y);
               // pc.printf("%f,%f,%f,%f,%f,%f,%d\r\n", estima_atitude.Phi, estima_atitude.Theta, estima_atitude.Psi, estima_atitude.P, estima_atitude.Q, estima_atitude.R, tempo.read_ms());

                cont_atitude.controle(phi_r, theta_r, estima_atitude.Phi, estima_atitude.Theta, estima_atitude.P, estima_atitude.Q, p_r, q_r);

                mixer.actuate(cont_atitude.f_x, cont_atitude.f_y, m * g);

            }
            
            tempo.reset();

        }

        mixer.estado_seguro();
        inicializa.aviso_sonoro();
        wait_ms(10);
        pc.printf("Fora do while principal \r\n");
        pc.printf("PITCH= %f, ROLL= %f, P= %f, q= %f, r= %f\r\n", estima_atitude.Theta, estima_atitude.Phi, estima_atitude.P, estima_atitude.Q, estima_atitude.R);
    }
}

void callback()
{
    flag = true;

}
