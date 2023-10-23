################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/u8glib/chessengine.c \
../Drivers/u8glib/u8g_bitmap.c \
../Drivers/u8glib/u8g_circle.c \
../Drivers/u8glib/u8g_clip.c \
../Drivers/u8glib/u8g_com_api.c \
../Drivers/u8glib/u8g_com_api_16gr.c \
../Drivers/u8glib/u8g_com_i2c.c \
../Drivers/u8glib/u8g_com_io.c \
../Drivers/u8glib/u8g_com_linux_ssd_i2c.c \
../Drivers/u8glib/u8g_com_msp430_hw_spi.c \
../Drivers/u8glib/u8g_com_null.c \
../Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.c \
../Drivers/u8glib/u8g_com_raspberrypi_hw_spi.c \
../Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.c \
../Drivers/u8glib/u8g_com_std_sw_spi.c \
../Drivers/u8glib/u8g_cursor.c \
../Drivers/u8glib/u8g_delay.c \
../Drivers/u8glib/u8g_dev_null.c \
../Drivers/u8glib/u8g_dev_ssd1306_128x64.c \
../Drivers/u8glib/u8g_ellipse.c \
../Drivers/u8glib/u8g_font.c \
../Drivers/u8glib/u8g_line.c \
../Drivers/u8glib/u8g_ll_api.c \
../Drivers/u8glib/u8g_page.c \
../Drivers/u8glib/u8g_pb.c \
../Drivers/u8glib/u8g_pb14v1.c \
../Drivers/u8glib/u8g_pb16h1.c \
../Drivers/u8glib/u8g_pb16h2.c \
../Drivers/u8glib/u8g_pb16v1.c \
../Drivers/u8glib/u8g_pb16v2.c \
../Drivers/u8glib/u8g_pb32h1.c \
../Drivers/u8glib/u8g_pb8h1.c \
../Drivers/u8glib/u8g_pb8h1f.c \
../Drivers/u8glib/u8g_pb8h2.c \
../Drivers/u8glib/u8g_pb8h8.c \
../Drivers/u8glib/u8g_pb8v1.c \
../Drivers/u8glib/u8g_pb8v2.c \
../Drivers/u8glib/u8g_pbxh16.c \
../Drivers/u8glib/u8g_pbxh24.c \
../Drivers/u8glib/u8g_polygon.c \
../Drivers/u8glib/u8g_rect.c \
../Drivers/u8glib/u8g_rot.c \
../Drivers/u8glib/u8g_scale.c \
../Drivers/u8glib/u8g_state.c \
../Drivers/u8glib/u8g_u16toa.c \
../Drivers/u8glib/u8g_u8toa.c \
../Drivers/u8glib/u8g_virtual_screen.c 

OBJS += \
./Drivers/u8glib/chessengine.o \
./Drivers/u8glib/u8g_bitmap.o \
./Drivers/u8glib/u8g_circle.o \
./Drivers/u8glib/u8g_clip.o \
./Drivers/u8glib/u8g_com_api.o \
./Drivers/u8glib/u8g_com_api_16gr.o \
./Drivers/u8glib/u8g_com_i2c.o \
./Drivers/u8glib/u8g_com_io.o \
./Drivers/u8glib/u8g_com_linux_ssd_i2c.o \
./Drivers/u8glib/u8g_com_msp430_hw_spi.o \
./Drivers/u8glib/u8g_com_null.o \
./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.o \
./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.o \
./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.o \
./Drivers/u8glib/u8g_com_std_sw_spi.o \
./Drivers/u8glib/u8g_cursor.o \
./Drivers/u8glib/u8g_delay.o \
./Drivers/u8glib/u8g_dev_null.o \
./Drivers/u8glib/u8g_dev_ssd1306_128x64.o \
./Drivers/u8glib/u8g_ellipse.o \
./Drivers/u8glib/u8g_font.o \
./Drivers/u8glib/u8g_line.o \
./Drivers/u8glib/u8g_ll_api.o \
./Drivers/u8glib/u8g_page.o \
./Drivers/u8glib/u8g_pb.o \
./Drivers/u8glib/u8g_pb14v1.o \
./Drivers/u8glib/u8g_pb16h1.o \
./Drivers/u8glib/u8g_pb16h2.o \
./Drivers/u8glib/u8g_pb16v1.o \
./Drivers/u8glib/u8g_pb16v2.o \
./Drivers/u8glib/u8g_pb32h1.o \
./Drivers/u8glib/u8g_pb8h1.o \
./Drivers/u8glib/u8g_pb8h1f.o \
./Drivers/u8glib/u8g_pb8h2.o \
./Drivers/u8glib/u8g_pb8h8.o \
./Drivers/u8glib/u8g_pb8v1.o \
./Drivers/u8glib/u8g_pb8v2.o \
./Drivers/u8glib/u8g_pbxh16.o \
./Drivers/u8glib/u8g_pbxh24.o \
./Drivers/u8glib/u8g_polygon.o \
./Drivers/u8glib/u8g_rect.o \
./Drivers/u8glib/u8g_rot.o \
./Drivers/u8glib/u8g_scale.o \
./Drivers/u8glib/u8g_state.o \
./Drivers/u8glib/u8g_u16toa.o \
./Drivers/u8glib/u8g_u8toa.o \
./Drivers/u8glib/u8g_virtual_screen.o 

C_DEPS += \
./Drivers/u8glib/chessengine.d \
./Drivers/u8glib/u8g_bitmap.d \
./Drivers/u8glib/u8g_circle.d \
./Drivers/u8glib/u8g_clip.d \
./Drivers/u8glib/u8g_com_api.d \
./Drivers/u8glib/u8g_com_api_16gr.d \
./Drivers/u8glib/u8g_com_i2c.d \
./Drivers/u8glib/u8g_com_io.d \
./Drivers/u8glib/u8g_com_linux_ssd_i2c.d \
./Drivers/u8glib/u8g_com_msp430_hw_spi.d \
./Drivers/u8glib/u8g_com_null.d \
./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.d \
./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.d \
./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.d \
./Drivers/u8glib/u8g_com_std_sw_spi.d \
./Drivers/u8glib/u8g_cursor.d \
./Drivers/u8glib/u8g_delay.d \
./Drivers/u8glib/u8g_dev_null.d \
./Drivers/u8glib/u8g_dev_ssd1306_128x64.d \
./Drivers/u8glib/u8g_ellipse.d \
./Drivers/u8glib/u8g_font.d \
./Drivers/u8glib/u8g_line.d \
./Drivers/u8glib/u8g_ll_api.d \
./Drivers/u8glib/u8g_page.d \
./Drivers/u8glib/u8g_pb.d \
./Drivers/u8glib/u8g_pb14v1.d \
./Drivers/u8glib/u8g_pb16h1.d \
./Drivers/u8glib/u8g_pb16h2.d \
./Drivers/u8glib/u8g_pb16v1.d \
./Drivers/u8glib/u8g_pb16v2.d \
./Drivers/u8glib/u8g_pb32h1.d \
./Drivers/u8glib/u8g_pb8h1.d \
./Drivers/u8glib/u8g_pb8h1f.d \
./Drivers/u8glib/u8g_pb8h2.d \
./Drivers/u8glib/u8g_pb8h8.d \
./Drivers/u8glib/u8g_pb8v1.d \
./Drivers/u8glib/u8g_pb8v2.d \
./Drivers/u8glib/u8g_pbxh16.d \
./Drivers/u8glib/u8g_pbxh24.d \
./Drivers/u8glib/u8g_polygon.d \
./Drivers/u8glib/u8g_rect.d \
./Drivers/u8glib/u8g_rot.d \
./Drivers/u8glib/u8g_scale.d \
./Drivers/u8glib/u8g_state.d \
./Drivers/u8glib/u8g_u16toa.d \
./Drivers/u8glib/u8g_u8toa.d \
./Drivers/u8glib/u8g_virtual_screen.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/u8glib/%.o Drivers/u8glib/%.su Drivers/u8glib/%.cyclo: ../Drivers/u8glib/%.c Drivers/u8glib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"X:/Users/DaniBrem/Desktop/ProyectInSTM32/Profe/TP1_RTOS_OLED_ADC_4_Channel/Drivers/u8glib" -I"X:/Users/DaniBrem/Desktop/ProyectInSTM32/Profe/TP1_RTOS_OLED_ADC_4_Channel/Drivers/fntsrc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-u8glib

clean-Drivers-2f-u8glib:
	-$(RM) ./Drivers/u8glib/chessengine.cyclo ./Drivers/u8glib/chessengine.d ./Drivers/u8glib/chessengine.o ./Drivers/u8glib/chessengine.su ./Drivers/u8glib/u8g_bitmap.cyclo ./Drivers/u8glib/u8g_bitmap.d ./Drivers/u8glib/u8g_bitmap.o ./Drivers/u8glib/u8g_bitmap.su ./Drivers/u8glib/u8g_circle.cyclo ./Drivers/u8glib/u8g_circle.d ./Drivers/u8glib/u8g_circle.o ./Drivers/u8glib/u8g_circle.su ./Drivers/u8glib/u8g_clip.cyclo ./Drivers/u8glib/u8g_clip.d ./Drivers/u8glib/u8g_clip.o ./Drivers/u8glib/u8g_clip.su ./Drivers/u8glib/u8g_com_api.cyclo ./Drivers/u8glib/u8g_com_api.d ./Drivers/u8glib/u8g_com_api.o ./Drivers/u8glib/u8g_com_api.su ./Drivers/u8glib/u8g_com_api_16gr.cyclo ./Drivers/u8glib/u8g_com_api_16gr.d ./Drivers/u8glib/u8g_com_api_16gr.o ./Drivers/u8glib/u8g_com_api_16gr.su ./Drivers/u8glib/u8g_com_i2c.cyclo ./Drivers/u8glib/u8g_com_i2c.d ./Drivers/u8glib/u8g_com_i2c.o ./Drivers/u8glib/u8g_com_i2c.su ./Drivers/u8glib/u8g_com_io.cyclo ./Drivers/u8glib/u8g_com_io.d ./Drivers/u8glib/u8g_com_io.o ./Drivers/u8glib/u8g_com_io.su ./Drivers/u8glib/u8g_com_linux_ssd_i2c.cyclo ./Drivers/u8glib/u8g_com_linux_ssd_i2c.d ./Drivers/u8glib/u8g_com_linux_ssd_i2c.o ./Drivers/u8glib/u8g_com_linux_ssd_i2c.su ./Drivers/u8glib/u8g_com_msp430_hw_spi.cyclo ./Drivers/u8glib/u8g_com_msp430_hw_spi.d ./Drivers/u8glib/u8g_com_msp430_hw_spi.o ./Drivers/u8glib/u8g_com_msp430_hw_spi.su ./Drivers/u8glib/u8g_com_null.cyclo ./Drivers/u8glib/u8g_com_null.d ./Drivers/u8glib/u8g_com_null.o ./Drivers/u8glib/u8g_com_null.su ./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.cyclo ./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.d ./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.o ./Drivers/u8glib/u8g_com_psoc5_ssd_hw_parallel.su ./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.cyclo ./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.d ./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.o ./Drivers/u8glib/u8g_com_raspberrypi_hw_spi.su ./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.cyclo ./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.d ./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.o ./Drivers/u8glib/u8g_com_raspberrypi_ssd_i2c.su ./Drivers/u8glib/u8g_com_std_sw_spi.cyclo ./Drivers/u8glib/u8g_com_std_sw_spi.d ./Drivers/u8glib/u8g_com_std_sw_spi.o ./Drivers/u8glib/u8g_com_std_sw_spi.su ./Drivers/u8glib/u8g_cursor.cyclo ./Drivers/u8glib/u8g_cursor.d ./Drivers/u8glib/u8g_cursor.o ./Drivers/u8glib/u8g_cursor.su ./Drivers/u8glib/u8g_delay.cyclo ./Drivers/u8glib/u8g_delay.d ./Drivers/u8glib/u8g_delay.o ./Drivers/u8glib/u8g_delay.su ./Drivers/u8glib/u8g_dev_null.cyclo ./Drivers/u8glib/u8g_dev_null.d ./Drivers/u8glib/u8g_dev_null.o ./Drivers/u8glib/u8g_dev_null.su ./Drivers/u8glib/u8g_dev_ssd1306_128x64.cyclo ./Drivers/u8glib/u8g_dev_ssd1306_128x64.d ./Drivers/u8glib/u8g_dev_ssd1306_128x64.o ./Drivers/u8glib/u8g_dev_ssd1306_128x64.su ./Drivers/u8glib/u8g_ellipse.cyclo ./Drivers/u8glib/u8g_ellipse.d ./Drivers/u8glib/u8g_ellipse.o ./Drivers/u8glib/u8g_ellipse.su ./Drivers/u8glib/u8g_font.cyclo ./Drivers/u8glib/u8g_font.d ./Drivers/u8glib/u8g_font.o ./Drivers/u8glib/u8g_font.su ./Drivers/u8glib/u8g_line.cyclo ./Drivers/u8glib/u8g_line.d ./Drivers/u8glib/u8g_line.o ./Drivers/u8glib/u8g_line.su ./Drivers/u8glib/u8g_ll_api.cyclo ./Drivers/u8glib/u8g_ll_api.d ./Drivers/u8glib/u8g_ll_api.o ./Drivers/u8glib/u8g_ll_api.su ./Drivers/u8glib/u8g_page.cyclo ./Drivers/u8glib/u8g_page.d ./Drivers/u8glib/u8g_page.o ./Drivers/u8glib/u8g_page.su ./Drivers/u8glib/u8g_pb.cyclo ./Drivers/u8glib/u8g_pb.d ./Drivers/u8glib/u8g_pb.o ./Drivers/u8glib/u8g_pb.su ./Drivers/u8glib/u8g_pb14v1.cyclo ./Drivers/u8glib/u8g_pb14v1.d ./Drivers/u8glib/u8g_pb14v1.o ./Drivers/u8glib/u8g_pb14v1.su ./Drivers/u8glib/u8g_pb16h1.cyclo ./Drivers/u8glib/u8g_pb16h1.d ./Drivers/u8glib/u8g_pb16h1.o ./Drivers/u8glib/u8g_pb16h1.su ./Drivers/u8glib/u8g_pb16h2.cyclo ./Drivers/u8glib/u8g_pb16h2.d ./Drivers/u8glib/u8g_pb16h2.o ./Drivers/u8glib/u8g_pb16h2.su ./Drivers/u8glib/u8g_pb16v1.cyclo ./Drivers/u8glib/u8g_pb16v1.d ./Drivers/u8glib/u8g_pb16v1.o ./Drivers/u8glib/u8g_pb16v1.su ./Drivers/u8glib/u8g_pb16v2.cyclo ./Drivers/u8glib/u8g_pb16v2.d ./Drivers/u8glib/u8g_pb16v2.o ./Drivers/u8glib/u8g_pb16v2.su ./Drivers/u8glib/u8g_pb32h1.cyclo ./Drivers/u8glib/u8g_pb32h1.d ./Drivers/u8glib/u8g_pb32h1.o ./Drivers/u8glib/u8g_pb32h1.su ./Drivers/u8glib/u8g_pb8h1.cyclo ./Drivers/u8glib/u8g_pb8h1.d ./Drivers/u8glib/u8g_pb8h1.o ./Drivers/u8glib/u8g_pb8h1.su ./Drivers/u8glib/u8g_pb8h1f.cyclo ./Drivers/u8glib/u8g_pb8h1f.d ./Drivers/u8glib/u8g_pb8h1f.o ./Drivers/u8glib/u8g_pb8h1f.su ./Drivers/u8glib/u8g_pb8h2.cyclo ./Drivers/u8glib/u8g_pb8h2.d ./Drivers/u8glib/u8g_pb8h2.o ./Drivers/u8glib/u8g_pb8h2.su ./Drivers/u8glib/u8g_pb8h8.cyclo ./Drivers/u8glib/u8g_pb8h8.d ./Drivers/u8glib/u8g_pb8h8.o ./Drivers/u8glib/u8g_pb8h8.su ./Drivers/u8glib/u8g_pb8v1.cyclo ./Drivers/u8glib/u8g_pb8v1.d ./Drivers/u8glib/u8g_pb8v1.o ./Drivers/u8glib/u8g_pb8v1.su ./Drivers/u8glib/u8g_pb8v2.cyclo ./Drivers/u8glib/u8g_pb8v2.d ./Drivers/u8glib/u8g_pb8v2.o ./Drivers/u8glib/u8g_pb8v2.su ./Drivers/u8glib/u8g_pbxh16.cyclo ./Drivers/u8glib/u8g_pbxh16.d ./Drivers/u8glib/u8g_pbxh16.o ./Drivers/u8glib/u8g_pbxh16.su ./Drivers/u8glib/u8g_pbxh24.cyclo ./Drivers/u8glib/u8g_pbxh24.d ./Drivers/u8glib/u8g_pbxh24.o ./Drivers/u8glib/u8g_pbxh24.su ./Drivers/u8glib/u8g_polygon.cyclo ./Drivers/u8glib/u8g_polygon.d ./Drivers/u8glib/u8g_polygon.o ./Drivers/u8glib/u8g_polygon.su ./Drivers/u8glib/u8g_rect.cyclo ./Drivers/u8glib/u8g_rect.d ./Drivers/u8glib/u8g_rect.o ./Drivers/u8glib/u8g_rect.su ./Drivers/u8glib/u8g_rot.cyclo ./Drivers/u8glib/u8g_rot.d ./Drivers/u8glib/u8g_rot.o ./Drivers/u8glib/u8g_rot.su ./Drivers/u8glib/u8g_scale.cyclo ./Drivers/u8glib/u8g_scale.d ./Drivers/u8glib/u8g_scale.o ./Drivers/u8glib/u8g_scale.su ./Drivers/u8glib/u8g_state.cyclo ./Drivers/u8glib/u8g_state.d ./Drivers/u8glib/u8g_state.o ./Drivers/u8glib/u8g_state.su ./Drivers/u8glib/u8g_u16toa.cyclo ./Drivers/u8glib/u8g_u16toa.d ./Drivers/u8glib/u8g_u16toa.o ./Drivers/u8glib/u8g_u16toa.su ./Drivers/u8glib/u8g_u8toa.cyclo
	-$(RM) ./Drivers/u8glib/u8g_u8toa.d ./Drivers/u8glib/u8g_u8toa.o ./Drivers/u8glib/u8g_u8toa.su ./Drivers/u8glib/u8g_virtual_screen.cyclo ./Drivers/u8glib/u8g_virtual_screen.d ./Drivers/u8glib/u8g_virtual_screen.o ./Drivers/u8glib/u8g_virtual_screen.su

.PHONY: clean-Drivers-2f-u8glib

