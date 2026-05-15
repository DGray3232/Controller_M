################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AHRSAlgorithms.c \
../Core/Src/bmx055.c \
../Core/Src/calculate_notch_coeffs.c \
../Core/Src/dma.c \
../Core/Src/globals.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/median_moving_average_filter.c \
../Core/Src/mtf02.c \
../Core/Src/optical_flow_compensation.c \
../Core/Src/pid.c \
../Core/Src/remote_control.c \
../Core/Src/remote_control_mavlink.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/vibration_analysis.c 

OBJS += \
./Core/Src/AHRSAlgorithms.o \
./Core/Src/bmx055.o \
./Core/Src/calculate_notch_coeffs.o \
./Core/Src/dma.o \
./Core/Src/globals.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/median_moving_average_filter.o \
./Core/Src/mtf02.o \
./Core/Src/optical_flow_compensation.o \
./Core/Src/pid.o \
./Core/Src/remote_control.o \
./Core/Src/remote_control_mavlink.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/vibration_analysis.o 

C_DEPS += \
./Core/Src/AHRSAlgorithms.d \
./Core/Src/bmx055.d \
./Core/Src/calculate_notch_coeffs.d \
./Core/Src/dma.d \
./Core/Src/globals.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/median_moving_average_filter.d \
./Core/Src/mtf02.d \
./Core/Src/optical_flow_compensation.d \
./Core/Src/pid.d \
./Core/Src/remote_control.d \
./Core/Src/remote_control_mavlink.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/vibration_analysis.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"D:/workspace_1.10.1/Controller_M/Core/Inc/mavlink/minimal" -I"D:/workspace_1.10.1/Controller_M/Core/Inc/mavlink/standard" -I"D:/workspace_1.10.1/Controller_M/Core/Inc/mavlink/common" -I"D:/workspace_1.10.1/Controller_M/Core/Inc/mavlink" -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -Wno-address-of-packed-member -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AHRSAlgorithms.d ./Core/Src/AHRSAlgorithms.o ./Core/Src/AHRSAlgorithms.su ./Core/Src/bmx055.d ./Core/Src/bmx055.o ./Core/Src/bmx055.su ./Core/Src/calculate_notch_coeffs.d ./Core/Src/calculate_notch_coeffs.o ./Core/Src/calculate_notch_coeffs.su ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/globals.d ./Core/Src/globals.o ./Core/Src/globals.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/median_moving_average_filter.d ./Core/Src/median_moving_average_filter.o ./Core/Src/median_moving_average_filter.su ./Core/Src/mtf02.d ./Core/Src/mtf02.o ./Core/Src/mtf02.su ./Core/Src/optical_flow_compensation.d ./Core/Src/optical_flow_compensation.o ./Core/Src/optical_flow_compensation.su ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/remote_control.d ./Core/Src/remote_control.o ./Core/Src/remote_control.su ./Core/Src/remote_control_mavlink.d ./Core/Src/remote_control_mavlink.o ./Core/Src/remote_control_mavlink.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/vibration_analysis.d ./Core/Src/vibration_analysis.o ./Core/Src/vibration_analysis.su

.PHONY: clean-Core-2f-Src

