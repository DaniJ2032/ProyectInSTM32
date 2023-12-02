/* INTRUMENTO VIRTUAL  2023 */

#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "stdbool.h"

/* Librerias de Usuario*/
#include "My_adc.h"
#include "My_PWM.h"
#include "printf_out.h"
#include "structrForFrames.h"


/* Pines para el mux */

#define PIN1 GPIO_PIN_2 // Pines de seleccion
#define PIN2 GPIO_PIN_3 // Pines de seleccion
#define PIN3 GPIO_PIN_4 // Pines de seleccion

/* Variables Globales */
charRxFrame_t rxDataFramechar = {0};
frame_t dataFrame = {0};

/* Funciones del usuario */
void SystemClock_Config(void);
void generateFrameTx(void);
void recivedFrameRx(void);
void digital_write(uint8_t digital_output);
bool sweepMux(TIM_HandleTypeDef *htim);
uint8_t codeCRC8(uint8_t *dataFrame, uint8_t longitud);
uint8_t digital_read(void);


int main(void) {

  //-- Inicializacion de bloques --
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();

  //-- Inicializamos un timer --
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxDataFramechar, sizeof(rxDataFramechar));

  PWM_init(&PWMHAN, PWM1_CH);
  PWM_init(&PWMHAN, PWM2_CH);	// Iniciamos PWM de TIM3


  while (1) {

		  generateFrameTx();
  }
}

void generateFrameTx() {
  static uint16_t lecturaAdc;
  static unsigned char count = 0;
  static unsigned char contador = 1;

  while (contador <= 8) {
    lecturaAdc = ADC_Read(); // Lectura del ADC
    switch (contador) {
      case 1: dataFrame.inA1 = lecturaAdc; break;
      case 2: dataFrame.inA2 = lecturaAdc; break;
      case 3: dataFrame.inA3 = lecturaAdc; break;
      case 4: dataFrame.inA4 = lecturaAdc; break;
      case 5: dataFrame.inA5 = lecturaAdc; break;
      case 6: dataFrame.inA6 = lecturaAdc; break;
      case 7: dataFrame.inA7 = lecturaAdc; break;
      case 8:
        dataFrame.inA8 = lecturaAdc;
        count++;
        dataFrame.start = 0x1B;
        dataFrame.count = count;
        dataFrame.outA1 = rxDataFramechar.tramaEnvio.outA1;		// Tiene que venir desde QT
        dataFrame.outA2 = rxDataFramechar.tramaEnvio.outA2;		// Tiene que venir desde QT
        dataFrame.insDig = 0xB5;//digital_read();  					// Digital_read() hace funcion
        dataFrame.outsDig = rxDataFramechar.tramaEnvio.outsDig;	// Tiene que venir desde QT

        dataFrame.crc8 = codeCRC8((uint8_t *)&dataFrame, sizeof(dataFrame) - 2);

        HAL_UART_Transmit(&huart1, (uint8_t *)&dataFrame, sizeof(dataFrame), 0XFFFF);
        HAL_UART_AbortTransmit(&huart1);
        break;
    }

    if (sweepMux(&htim2) == true) contador++;

  }

  if (contador == 9) contador = 1;
}

bool sweepMux(TIM_HandleTypeDef *htim) {  // Contador a una frecuencia de 12kHz
  static unsigned char contador = 1;

  if (htim->Instance == TIM2) {
	HAL_GPIO_WritePin(GPIOA, PIN1, (contador & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, PIN2, (contador & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, PIN3, (contador & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	contador++;
  }

  return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if (huart->Instance == USART1) {

	  /*-- Seteo de las salidas --*/
	  uint8_t CRCR8_Check = codeCRC8((uint8_t *)&rxDataFramechar.tramaEnvio, sizeof(charRxFrame_t) - 1);

	  if (CRCR8_Check == rxDataFramechar.tramaEnvio.crc8){
	  // Salidas digitales
	  digital_write(rxDataFramechar.tramaEnvio.outsDig);
	  // PWM salidas
	  pwm_valor((uint32_t *)&PWM1, rxDataFramechar.tramaEnvio.outA1);
	  pwm_valor((uint32_t *)&PWM2, rxDataFramechar.tramaEnvio.outA2);

	  } else {}


	  HAL_UART_AbortReceive(&huart1);
	  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxDataFramechar, sizeof(rxDataFramechar));

  }
}

uint8_t digital_read(void) {

    uint8_t inputVector = 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_IDR11) ? (1 << 0) : 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_IDR10) ? (1 << 1) : 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_IDR9)  ? (1 << 2) : 0;
    inputVector |= (GPIOA->IDR & GPIO_IDR_IDR8)  ? (1 << 3) : 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_IDR15) ? (1 << 4) : 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_IDR14) ? (1 << 5) : 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_IDR13) ? (1 << 6) : 0;
    inputVector |= (GPIOB->IDR & GPIO_IDR_IDR12) ? (1 << 7) : 0;
    return inputVector;
}


void digital_write(uint8_t digital_output) {

    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_5) | ((digital_output & 0x1) ? GPIO_PIN_10 : 0);
    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_6) | ((digital_output & 0x2) ? GPIO_PIN_5 : 0);
    GPIOA->ODR = (GPIOA->ODR & ~GPIO_PIN_7) | ((digital_output & 0x4) ? GPIO_PIN_4 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_0) | ((digital_output & 0x8) ? GPIO_PIN_10 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_1) | ((digital_output & 0x10) ? GPIO_PIN_8 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_2) | ((digital_output & 0x20) ? GPIO_PIN_9 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_10) | ((digital_output & 0x40) ? GPIO_PIN_7 : 0);
    GPIOB->ODR = (GPIOB->ODR & ~GPIO_PIN_11) | ((digital_output & 0x80) ? GPIO_PIN_6 : 0);

}

uint8_t codeCRC8(uint8_t *Frame, uint8_t longitud) {
  uint8_t polinomio_generador = 0x07;  // Polinomio generador CRC-8 (0x07)
  uint8_t reg_crc = 0;                  // Inicializa el registro CRC en cero

  for (uint8_t i = 0; i < longitud; i++) {  // Se recorre la estructura
    reg_crc ^= Frame[i];                    // Realiza un XOR con el byte actual

    for (uint8_t j = 0; j < 8; j++) {
      if (reg_crc & 0x80) {  // // Si el bit MSB = 1
        /*se realiza una operación XOR con el polinomio generador y se
         * desplaza el registro CRC un bit a la izquierda*/
        reg_crc = (reg_crc << 1) ^ polinomio_generador;

        // Si el MSB = 0 Simplemente se desplaza 1 bit a la izquierda el reg_crc
      } else
        reg_crc <<= 1;
    }
  }

  return reg_crc;
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

  while (1){}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

