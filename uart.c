#include "stm32f1xx_hal.h"
#include "uart.h"

UART_HandleTypeDef Uart;

void UART_Config(void)
{
	Uart.Instance        = USART1;

	Uart.Init.BaudRate   = UART_BAUDRATE;
	Uart.Init.WordLength = UART_WORDLENGTH_8B;
	Uart.Init.StopBits   = UART_STOPBITS_1;
	Uart.Init.Parity     = UART_PARITY_NONE;
	Uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart.Init.Mode       = UART_MODE_TX_RX;

	HAL_UART_Init(&Uart);

	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}
