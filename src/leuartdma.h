#ifndef SRC_LEUARTDMA_H_
#define SRC_LEUARTDMA_H_

#include"main.h"

void setupLeuart(void);
void setupDma(void);
void WIFI_Connect(void);
void Send_Command();
void wait();

/*wifi module */
char command[100];
int command_size;

#endif
