//main.c - Main program for a bare metal Raspberry Pi 3B interfacing with a custom DLS sensor
#include "RPi_DLS.h"

#define BUFF_SIZE 8000000 //Number of 16bit data points to save

/* UART MESSAGE FLAGS */
#define SIG_ACQUIRE 0x41 //A
#define SIG_SEND 0x53 //S
#define SIG_BUSY 0x42 //B
#define SIG_READY 0x52 //R 

int main(void){
    uartInit();
 //   uint16_t * data_buffer = (uint16_t *)calloc(BUFF_SIZE, sizeof(uint16_t)); // Unable to use stdlib
    uint16_t data_buffer [BUFF_SIZE];

    while(1){
        unsigned int msg = uartRead();
        if(msg == SIG_ACQUIRE){
            uartSend(SIG_BUSY);
            for(unsigned int i=0; i<BUFF_SIZE; i++)
            {
                while(dataClk()==0){;}
                while(dataClk()==1){;}
                data_buffer[i] = dataRead();
            }
        }
        else if(msg == SIG_SEND){
            for(unsigned int i=0; i<BUFF_SIZE; i++)
            {
                int byte = 0;
                for(int j=0; j<4; j++)
                {
                    byte = data_buffer[i] & 0xFF;
                    uartSend(byte);
                    data_buffer[i] = data_buffer[i] >> 8;

                }
            }
        }

        uartSend(SIG_READY);
    }
}
