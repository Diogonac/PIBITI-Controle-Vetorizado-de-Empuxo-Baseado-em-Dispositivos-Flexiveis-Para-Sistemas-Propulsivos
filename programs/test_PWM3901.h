#include "mbed.h"
#include "imports.h"

//============ Configura porta serial ===============
Serial bt(SERIAL_TX1, SERIAL_RX1, 9600); // Comunicação com USB TX, RX

// Define flow sensor object
PMW3901 flow(E_MOSI, E_MISO, E_SCK, E_CS1);

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
        wait(0.1);
    }
}