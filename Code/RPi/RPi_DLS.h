#include <stdint-gcc.h>

uint16_t dataRead(void);
int dataClk(void);
void uartInit(void);
unsigned int uartRead(void);
void uartSend(unsigned int c);
