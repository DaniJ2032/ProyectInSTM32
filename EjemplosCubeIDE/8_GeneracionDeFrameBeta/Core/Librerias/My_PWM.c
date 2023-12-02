#include <My_PWM.h>

void  PWM_init(TIM_HandleTypeDef *PWM,uint16_t canal)
{

HAL_TIM_PWM_Start(PWM, canal);

}

void pwm_valor(uint32_t *canal,uint8_t valor){
	//PWMTIM1->ARR   =100
	 //    ?         =valor

*canal=valor*(float)PWMTIM3->ARR/100;  // lo escala de 0 a 100%

}
