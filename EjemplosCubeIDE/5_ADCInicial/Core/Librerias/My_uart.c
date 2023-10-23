#include "My_uart.h"


void uartx_write(UART_HandleTypeDef *huart, uint8_t ch) {

HAL_UART_Transmit(huart, &ch, 1, 0xffff);
}

void uartx_write_text(UART_HandleTypeDef *huart, char *info){

  while(*info) uartx_write(huart, *info++); 
}

void uartx_write_IT(UART_HandleTypeDef *huart, uint8_t ch) {

HAL_UART_Transmit_IT(huart, &ch, 1);
}
void uartx_write_text_IT(UART_HandleTypeDef *huart, char *info){

  while(*info) uartx_write_IT(huart, *info++); 
} 