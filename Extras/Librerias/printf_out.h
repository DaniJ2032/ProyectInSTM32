#include "string.h"
#include "stdio.h"
#include "main.h"

#define uartx huart1   //   colocar el puerto serial a usar

extern UART_HandleTypeDef uartx;


int __io_putchar(int ch) {
 //ITM_SendChar(ch);//  si se quiere usar por debbuger
HAL_UART_Transmit(&uartx, (uint8_t *)&ch, 1, 0xFFFF);// printf al puerto serial sefinico
return(ch);
}



