#include "adc.h"

extern ADC_HandleTypeDef hadc1;

uint16_t ADC_Read()
{
	HAL_ADC_Start(&adc1);               // Inicio el ADC   
    HAL_ADC_PollForConversion(&adc1,1); // Conversión por interrup
    return (HAL_ADC_GetValue(&adc1));   // obtengo el valor del adc
}

