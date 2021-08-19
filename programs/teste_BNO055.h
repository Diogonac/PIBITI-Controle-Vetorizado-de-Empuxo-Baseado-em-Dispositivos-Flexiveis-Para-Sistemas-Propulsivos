#include "mbed.h"
#include "imports.h"


//============ Configura porta serial ===============
Serial pc (SERIAL_TX1, SERIAL_RX1); //Comunicação com USB TX, RX

//======= Configurações para reiniciar a placa ======
DigitalOut LED_RESET_PLACA(PA_5);
int cont_reset_placa = 0;

//================ Configura BNO055 =================
BNO055 BNO055(IMU_SDA, IMU_SCL); //Define as portas SDA/SCL do BNO055
Timer ciclo_BNO055;
Timer verifica_status;
int cont_reset_BNO055 = 1; //Contador para indicar a quantidade de vezes que o BNO055 foi reiniciado
int status_BNO055; //Armazena o status do BNO055
int status_sys_BNO055; //Armazena o status do sistema do BNO055
bool status_check_BNO055; //Armazena o status do BNO055
int status_selftest; //Armazena o valor do teste

//============= Configura Taxa de coleta =============
Timer t_dados; //Timer para taxa de aquisicao dos dados
Timer ciclo_programa; //Timer para verificar o tempo de ciclo do programa
int tx_aquisicao; //Quantos pontos serao plotados em um segundo
int t_amostragem; //valor do timer do Main

//============ Declara as funções utilizadas ==========
void configuracao_BNO055(void);
void calibracao_BNO055(void);
void reset_placa(void);

int main()
{

    pc.baud(115200); //Define a velocidade da porta USB

    t_dados.start();
    ciclo_programa.start();
    ciclo_BNO055.start();
    verifica_status.start();

    tx_aquisicao = 100;

    configuracao_BNO055();
    calibracao_BNO055();
    
    

    while(1) {

        if (t_amostragem >= 1000000/tx_aquisicao % status_BNO055 == 0 % status_check_BNO055 == true) {

            t_dados.reset();

//            BNO055.get_accel();
//            BNO055.get_gyro();
//            BNO055.get_lia();
//            BNO055.get_mag();
//            BNO055.get_grv();
//            
            BNO055.get_angles();
                        
//            pc.printf("%f,%f,%f,%f,%f,%f\r\n", BNO055.accel.x, BNO055.accel.y, BNO055.accel.z, BNO055.gyro.x, BNO055.gyro.y, BNO055.gyro.z);
//
            pc.printf("YAW= %f, ROLL= %f, PITCH= %f \r\n", BNO055.euler.yaw, BNO055.euler.roll, BNO055.euler.pitch);
//            pc.printf("AX= %f, AY= %f, AZ= %f \r\n", BNO055.accel.x, BNO055.accel.y, BNO055.accel.z);
//            pc.printf("AXL= %f, AYL= %f, AZL= %f \r\n", BNO055.lia.x, BNO055.lia.y, BNO055.lia.z);
//            pc.printf("MAGX= %f, MAGY= %f, MAGZ= %f \r\n", BNO055.mag.x, BNO055.mag.y, BNO055.mag.z);
//            pc.printf("GRVX= %f, GRVY= %f, GRVZ= %f \r\n", BNO055.gravity.x, BNO055.gravity.y, BNO055.gravity.z);
//            pc.printf("GIROX= %f, GIROY= %f, GIROZ= %f\r\n", BNO055.gyro.x, BNO055.gyro.y, BNO055.gyro.z);
//
//            
           pc.printf("Tempo de ciclo BNO055= %d \r\n", ciclo_BNO055.read_ms());

            ciclo_BNO055.reset();

        }

        if(verifica_status.read_ms() > 5000) {

            verifica_status.reset();
            status_BNO055 = BNO055.getSystemStatus(BNO055_SYS_ERR_ADDR); //Verifica o status do BNO055
            status_check_BNO055 = BNO055.check(); //Verifica se tem comunicação com o BNO055
            status_sys_BNO055 = BNO055.getSystemStatus(BNO055_SYS_STAT_ADDR); //Verifica o status do sistema BNO055

//            pc.printf("\r\n"); //Pula uma linha no leitor
//            pc.printf("Verifica status BNO055: %d\r\n", status_BNO055); //Indica 0 para operação nominal do BNO055
//            pc.printf("Verifica comunicacao BNO055: %d\r\n", status_check_BNO055); //Indica 1 para operação nominal do BNO055
//            pc.printf("Verifica o status do sistema do BNO055: %d\r\n", status_sys_BNO055); //Indica o status atual do BNO055

            while(status_BNO055 != 0 & status_check_BNO055 != true) {


                BNO055.reset(); //Reseta o BNO055
                status_BNO055 = BNO055.getSystemStatus(BNO055_SYS_ERR_ADDR); //Verifica o status do BNO055
                status_check_BNO055 = BNO055.check(); //Verifica se tem comunicação com o BNO055
                status_sys_BNO055 = BNO055.getSystemStatus(BNO055_SYS_STAT_ADDR); //Verifica o status do sistema BNO055
                cont_reset_BNO055++; //Soma 1 no contador

                pc.printf("Status BNO055: %d\r\n", status_BNO055); //Indica 0 para operação nominal do BNO055
                pc.printf("Count reset BNO055: %d\r\n", cont_reset_BNO055); //Indica que o BNO055 foi resetado
                pc.printf("Verifica comunicacao BNO055: %d\r\n", status_check_BNO055); //Indica 1 para operação nominal do BNO055
                pc.printf("Verifica o status do sistema do BNO055: %d\r\n", status_sys_BNO055); //Indica o status atual do BNO055
                if(status_BNO055 == 0 % status_check_BNO055 == true) {
                    //Caso o ciclo atual detectar conexão com o BNO055 (status_BNO055 = 1), as configuração iniciais são chamadas
                    configuracao_BNO055();
                    calibracao_BNO055();

                }

                if(cont_reset_BNO055 >= 41) reset_placa(); //Chama a função de reiniciar a placa depois de 40 tentativas

            }

        }

       // pc.printf("Tempo de ciclo do programa= %d \r\n", ciclo_programa.read_ms());
        ciclo_programa.reset();
    }
}

void configuracao_BNO055()
{

    //============= Configurações iniciais no BNO055 ================
    cont_reset_BNO055 = 1; //Reinicia o contador de reset
    BNO055.reset(); //Reseta o BNO055
    status_sys_BNO055 = BNO055.getSystemStatus(BNO055_SYS_STAT_ADDR); //Verifica o status do sistema BNO055
    status_BNO055 = BNO055.getSystemStatus(BNO055_SYS_ERR_ADDR); //Verifica o status do BNO055
    status_check_BNO055 = BNO055.check(); //Verifica se tem comunicação com o BNO055

    pc.printf("Status BNO055: %d\r\n", status_BNO055); //Indica 0 para operação nominal do BNO055
    pc.printf("Verifica comunicacao BNO055: %d\r\n", status_check_BNO055); //Indica 1 para operação nominal do BNO055
    pc.printf("Verifica o status do sistema do BNO055: %d\r\n", status_sys_BNO055); //Indica o status atual do BNO055

    BNO055.SetExternalCrystal(true); //Indica a existencia de um cristal externo
    wait_ms(675);
    BNO055.setpowermode(POWER_MODE_NORMAL); //Define o modo de alimentação do BNO055
    wait_ms(25); //Aguarda a troca de modo de alimentação
    BNO055.setmode(OPERATION_MODE_CONFIG); //Configura o modo padrão para iniciar a calibração
    wait_ms(25); //Aguarda o BNO055 trocar de modo de operação

//============= Unidade das variáveis do BNO055 ================
    BNO055.set_accel_units(MPERSPERS); // m/s2
    BNO055.set_anglerate_units(DEG_PER_SEC); // graus°/s
    BNO055.set_angle_units(DEGREES); // graus°
    BNO055.set_temp_units(CENTIGRADE); // °C
    BNO055.set_orientation(ANDROID); // Sentido de rotação ANDROID = Regra da mão direita

}

void calibracao_BNO055()
{

//=================== Calibração do BNO055 =====================
//    BNO055.setmode(OPERATION_MODE_CONFIG); //Configura o modo padrão para iniciar a calibração
//    wait_ms(25); //Aguarda o BNO055 trocar de modo de operação
//    BNO055.read_calibration_data(); //Calibração dos sensores
//    wait_ms(10);
//    BNO055.get_calib();
//
//    while(BNO055.calib == 0) {
//
//        BNO055.get_calib();
//        pc.printf("Leitura de calibracao: %d \r\n", BNO055.calib); //Realiza um selftest no BNO055
//
//    }
//
//    BNO055.write_calibration_data(); //Calibração dos sensores
//    wait_ms(10);
    status_selftest = BNO055.getSystemStatus(BNO055_SELFTEST_RESULT_ADDR); //15 = todos os sensores então OK
    pc.printf("SelfTest Status: %d \r\n", status_selftest); //Realiza um selftest no BNO055 
    //BNO055.setmode(OPERATION_MODE_IMUPLUS); //Configura o mode de fusão entre acelerômetro e giroscópio taxa de atualização máxima: 100Hz (Relative orientation)
    BNO055.setmode(OPERATION_MODE_ACCGYRO); //Configura o modo de fusão para ler todos os sensores a um taxa de atualização máxima: 100Hz (Absolute orientation)
    wait_ms(25); //Aguarda o BNO055 trocar de modo de operação
    BNO055.config_BW(ACCEL_CONFIG, BNO055_ACCEL_BW_15_63HZ);
    wait_ms(25);
    BNO055.config_BW(GYRO_CONFIG, BNO055_GYRO_BW_12HZ);
    wait_ms(25);

}

void reset_placa()
{

//============ Indicador luminoso de reset da placa =============
    for (cont_reset_placa = 0; cont_reset_placa < 5; cont_reset_placa++) {

        LED_RESET_PLACA = 1;
        wait_ms(200);
        LED_RESET_PLACA = 0;
        wait_ms(200);

    }

    NVIC_SystemReset(); //Função que força o reset da placa

}