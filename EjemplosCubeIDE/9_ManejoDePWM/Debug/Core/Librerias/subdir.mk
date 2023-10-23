################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Librerias/My_adc.c \
../Core/Librerias/PWM.c 

OBJS += \
./Core/Librerias/My_adc.o \
./Core/Librerias/PWM.o 

C_DEPS += \
./Core/Librerias/My_adc.d \
./Core/Librerias/PWM.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Librerias/%.o Core/Librerias/%.su Core/Librerias/%.cyclo: ../Core/Librerias/%.c Core/Librerias/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Librerias -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Librerias

clean-Core-2f-Librerias:
	-$(RM) ./Core/Librerias/My_adc.cyclo ./Core/Librerias/My_adc.d ./Core/Librerias/My_adc.o ./Core/Librerias/My_adc.su ./Core/Librerias/PWM.cyclo ./Core/Librerias/PWM.d ./Core/Librerias/PWM.o ./Core/Librerias/PWM.su

.PHONY: clean-Core-2f-Librerias

