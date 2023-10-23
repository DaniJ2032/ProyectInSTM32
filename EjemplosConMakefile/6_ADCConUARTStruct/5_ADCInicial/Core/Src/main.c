/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "My_adc.h"
#include "My_uart.h"


// Declaracion de estructuras y uniones 
typedef struct {
  // Byte de caebcera.
  uint8_t start; 
  // Contador.  
  uint8_t count; 
  // 8 entradas analógicas (12 bits).
  uint16_t inA1; 
  uint16_t inA2;
  uint16_t inA3;
  uint16_t inA4;
  uint16_t inA5;
  uint16_t inA6;
  uint16_t inA7;
  uint16_t inA8;
  // 2 salidas analógicas.
  uint16_t outA1; 
  uint16_t outA2;
  // 8 entradas digitales.
  uint8_t insDig;  
  // 8 salidas digitales.
  uint8_t outsDig; 

} frame_t; //Fin de struct

typedef union
{
  frame_t frameEntrada;
  char    frameEntradaChar[24];
} charFrame_t; //Fin de union


void SystemClock_Config(void);

int main(void)
{
  //--Inicializacion de bloques-- 
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  // --Variables del sistema--
  charFrame_t dataFrame;
  uint16_t lecturaAdc;


  while (1){
    //REVISAR
    lecturaAdc = ADC_Read(); //Lectura del ADC
    // Utilizar snprintf para convertir el uint16_t a char
    dataFrame.frameEntrada.start = 0x1B;
    dataFrame.frameEntrada.count = 0xA;
    dataFrame.frameEntrada.inA1 = 0x01;
    dataFrame.frameEntrada.inA2 = 0x01;
    dataFrame.frameEntrada.inA3 = 0x01;
    dataFrame.frameEntrada.inA4 = 0x01;
    dataFrame.frameEntrada.inA5 = 0x01;
    dataFrame.frameEntrada.inA6 = 0x01;
    dataFrame.frameEntrada.inA7 = 0x01;
    dataFrame.frameEntrada.inA8 = 0x01;

    dataFrame.frameEntrada.outA1 = 0x02;
    dataFrame.frameEntrada.outA2 = 0x02;

    dataFrame.frameEntrada.insDig =  0x05;
    dataFrame.frameEntrada.outsDig = 0x05;


    uartx_write_text(&huart1, (char *)&dataFrame); 
    // while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {}

    // Esperar un tiempo antes de enviar el siguiente mensaje
    HAL_Delay(500);
    // char result_str[10];
    // snprintf(result_str, sizeof(result_str), "%u\r\n", lecturaAdc);    
    // uartx_write_text(&huart1, result_str);
  }
 
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1){
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
