#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1, 115200); //Comunicação com USB TX, RX

// Imports
Actuators act;
ControladorAtitude att_cont;
EstimadorAtitude att_est;
Initialization init;

// Ticker para taxa de amostragem
Ticker amostragem;

Timer tempo;

// Flag de segurança
bool flag;

// Variáveis de referencia
double phi_r, theta_r, p_r, q_r;


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
    act.servo_test(105.0, 75.0, 1.0);
    att_est.init();
    
    // init.verify(act.init_servos, att_est.verifica_imu);
    

    //Definição da taxa de amostragem
    amostragem.attach(&callback, dt);

    // while(init.sistema_verificado) {

        att_est.estimate();

        //p --> wx | q -- > wy | r --> wz
        while(true){//abs(att_est.Phi) <= pi /12.0 && abs(att_est.Theta) <= pi /12.0 && abs(att_est.P) <= pi/4 && abs (att_est.Q) <= pi/4 && abs(att_est.R) <= pi/4) {

            if(flag) {
                
                flag = false;

                att_est.estimate();

                //pc.printf("%f %f\r\n", estima_atitude.P, estima_atitude.Q);

                //pc.printf("YAW= %f, ROLL= %f, PITCH= %f, GIROX= %f, GIROY= %f \r\n", BNO055.euler.yaw, BNO055.euler.roll, BNO055.euler.pitch, BNO055.gyro.x, BNO055.gyro.y);
               //pc.printf("P_ref= %f %f %f,%f,%f,%f,%d\r\n", estima_atitude.Phi, estima_atitude.Theta, estima_atitude.Psi, estima_atitude.P, estima_atitude.Q, estima_atitude.R, tempo.read_ms());

               att_cont.control(phi_r, theta_r, att_est.Phi, att_est.Theta, att_est.P, att_est.Q, p_r, q_r);

                act.actuate_servos(att_cont.f_x, att_cont.f_y, m * g);
                // pc.printf("THETA= %f, PHI= %f, FX= %f, FY= %f\r\n", att_est.Theta*180.0/pi, att_est.Phi*180.0/pi, att_cont.f_x, att_cont.f_y);

            }
            
            tempo.reset();

        }

        act.safe_state();
        init.arm();
        att_est.estimate();
        wait_ms(10);
        pc.printf("Fora do while principal \r\n");
        pc.printf("PITCH= %f, ROLL= %f, P= %f, q= %f, r= %f\r\n", att_est.Theta, att_est.Phi, att_est.P, att_est.Q, att_est.R);
    }
// }

void callback()
{
    flag = true;

}
