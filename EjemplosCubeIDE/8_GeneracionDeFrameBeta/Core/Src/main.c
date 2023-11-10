/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include <stdbool.h>

#include "My_adc.h"
#include "printf_out.h"
#include "structrForFrames.h"

#define PIN1 GPIO_PIN_0 // Pines de seleccion
#define PIN2 GPIO_PIN_1 // Pines de seleccion
#define PIN3 GPIO_PIN_2 // Pines de seleccion


void SystemClock_Config(void);



bool sweepMux(TIM_HandleTypeDef *htim);

charRxFrame_t ProcessReceivedFrame(void);

void generateFrameTx(charRxFrame_t rxDataFramechar);


uint8_t codeCRC8(uint8_t *dataFrame, uint8_t longitud);

int main(void) {

  //-- Inicializacion de bloques --
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  // Inicializamos un timer
  HAL_TIM_Base_Start_IT(&htim2);

  charRxFrame_t rxDataFramechar={0};

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxDataFramechar,sizeof(rxDataFramechar));

  while (1){


	  //FUNCIONAAAAAAAAAAAAAAAA ANALIZAR
//	  rxDataFramechar = ProcessReceivedFrame();
//	  HAL_UART_Receive_IT(&huart1, rxDataFramechar.tramaEnvioChar,sizeof(rxDataFramechar));
	  generateFrameTx(rxDataFramechar);

//	  HAL_UART_Receive_DMA(&huart1, (uint8_t*)&dato, sizeof(dato));
//	  rxDataFramechar.tramaEnvioChar[i] = dato;
//	  generateFrameTx(rxDataFramechar);
//
//	  i++;
//	  if(i>=6) i =0;

  }

}

//charRxFrame_t ProcessReceivedFrame(void) {

//	static uint8_t frameIndex = 0;
////	static uint8_t frameReceived = 0;
//	static charRxFrame_t rxDataFramechar ={0};
//	uint8_t dato = 0x1B;
//
//    static enum {
//        WAIT_START,
//        RECEIVING_FRAME,
//    } state = WAIT_START;
//
//    while(frameIndex <= sizeof(rxDataFramechar.tramaEnvioChar)){
//		HAL_UART_Receive_DMA(&huart1, (uint8_t*)&dato, sizeof(dato));
//
//		switch (state) {
//			case WAIT_START:
//				if (dato == 0x1B) {
//					// Se ha detectado la cabecera, cambia al estado de recepción.
//					state = RECEIVING_FRAME;
//					rxDataFramechar.tramaEnvioChar[frameIndex] = dato;
//					frameIndex = 0;
//					frameIndex++;
////					dato =0;
//				}
//				break;
//
//			case RECEIVING_FRAME:
//				// Almacena el byte en el búfer temporal.
//				rxDataFramechar.tramaEnvioChar[frameIndex] = dato;
//				frameIndex++;
//
////				if (frameIndex >= sizeof(rxDataFramechar.tramaEnvioChar)) {}
//					// La estructura está completa, puedes procesarla.
////					frameIndex = 0;
////					state = WAIT_START;
//	               // Activa la bandera para indicar que se ha recibido un frame completo.
//
//				break;
//			default: break;
//		}
//    }
//	frameIndex = 0;
//	state = WAIT_START;


//	HAL_UART_Receive_DMA(&huart1, (uint8_t*)&rxDataFramechar.tramaEnvioChar, sizeof(rxDataFramechar));
//
//	return rxDataFramechar;
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	static charRxFrame_t rxDataFramechar;

	if (huart->Instance == USART1){


	}

//	// Verifico la trama recibida
//	uint8_t crc_check = codeCRC8((uint8_t *)&rxDataFramechar, sizeof(rxDataFramechar)-2);
//
//	if (crc_check != 0){
//
//		rxDataFramechar.tramaEnvio.outA1 = 0x1;
//		rxDataFramechar.tramaEnvio.outA2 = 0x2;
//		rxDataFramechar.tramaEnvio.outsDig = 0x3;
//		//ACA DEBERIA EJECUTAR LA ASIGNACION DE LOS PWM y salidas Dig.
//		// Ó irme a otra funcion.....

//	}

	// Recibimos por puerto serie el frame enviado por QT
	//ANDA MAL ESTO!!!!!!!!!! REVISAR SOLO RECIBO UNA VEZ
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxDataFramechar,sizeof(rxDataFramechar));
}

void generateFrameTx(charRxFrame_t rxDataFramechar){

	static frame_t			dataFrame = {0};
	static uint16_t			lecturaAdc;
	static unsigned char	count = 0;
	static unsigned char	contador = 1;
	//REVISAR
	uint8_t crc_check = codeCRC8((uint8_t *)&rxDataFramechar, sizeof(rxDataFramechar)-1);
	rxDataFramechar.tramaEnvio.outA1 = crc_check;

			while(contador<=8){

				lecturaAdc = ADC_Read(); //Lectura del ADC
				switch (contador){

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
						dataFrame.insDig =  0xDE;								//Digital_read() hace funcion
						dataFrame.outsDig = rxDataFramechar.tramaEnvio.outsDig;	// Tiene que venir desde QT

						dataFrame.crc8 = codeCRC8((uint8_t *)&dataFrame, sizeof(dataFrame)-2);

						HAL_UART_Transmit(&huart1, (uint8_t *)&dataFrame, sizeof(dataFrame),0xFFFF);

				  //	printf(" Trama Recibida: %u,%u,%u,%u,%u,%u", rxDataFrame.start, rxDataFrame.count,
				  //		  rxDataFrame.outA1, rxDataFrame.outA2, rxDataFrame.outsDig, rxDataFrame.crc8);

			//			printf("Trama enviada: %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \r\n", dataFrame.start, dataFrame.count,
			//					dataFrame.inA1, dataFrame.inA2, dataFrame.inA3, dataFrame.inA4, dataFrame.inA5,
			//					dataFrame.inA6, dataFrame.inA7, dataFrame.inA8, dataFrame.outA1, dataFrame.outA2,
			//					dataFrame.insDig,dataFrame.outsDig, dataFrame.crc8);
						break;
				}
				if (sweepMux(&htim2) == true) contador++;
			}

			if (contador == 9 ) contador =1;

}

bool sweepMux(TIM_HandleTypeDef *htim){	//Contador a una frec de 12kHz

	static unsigned char contador = 1;

	if (htim->Instance == TIM2){

        HAL_GPIO_WritePin(GPIOB, PIN1, (contador & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, PIN2, (contador & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, PIN3, (contador & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	contador++;
	}

	return true;
}


uint8_t codeCRC8(uint8_t *Frame, uint8_t longitud) {

    uint8_t polinomio_generador = 0x07; // Polinomio generador CRC-8 (0x07)
    uint8_t reg_crc = 0; // Inicializa el registro CRC en cero

    for (uint8_t i = 0; i < longitud; i++) { // Se recorre la estructura
        reg_crc ^= Frame[i]; // Realiza un XOR con el byte actual

        for (uint8_t j = 0; j < 8; j++) {

            if (reg_crc & 0x80) {	// // Si el bit MSB = 1
				/*se realiza una operación XOR con el polinomio generador y se
				 * desplaza el registro CRC un bit a la izquierda*/
                reg_crc = (reg_crc << 1) ^ polinomio_generador;

             // Si el MSB = 0 Simplemente se desplaza 1 bit a la izquierda el reg_crc
            }else reg_crc <<= 1;
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

