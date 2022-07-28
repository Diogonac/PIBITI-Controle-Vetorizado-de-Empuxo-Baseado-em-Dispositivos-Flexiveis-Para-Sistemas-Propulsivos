#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação com USB TX, RX

// Define flow sensor object
PMW3901 flow(E_MOSI, E_MISO, E_SCK, E_CS1);

    
PwmOut my1(D6);
PwmOut my2(D5);
PwmOut my3(D4);
PwmOut my4(D3);

// Main program
int main()
{
    // Initialize flow sensor object
    flow.init();
    // Print flow readings every 0.1s
    while(true) 
    {
        flow.read();
        bt.printf("Flow [px]: %f %f \n", flow.px, flow.py);
        my1 = 0.5;
        my2 = 0.5;
        my3 = 0.5;
        my4 = 0.5;
        wait(0.1);
    }
}