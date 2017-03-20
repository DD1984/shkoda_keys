#ifndef __UART_H__
#define __UART_H__

#define UART_BAUDRATE 115200

extern UART_HandleTypeDef Uart;

void UART_Config(void);

#endif
