################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HCSR04/HCSR04.c \
../Core/HCSR04/HCSR04_cfg.c 

OBJS += \
./Core/HCSR04/HCSR04.o \
./Core/HCSR04/HCSR04_cfg.o 

C_DEPS += \
./Core/HCSR04/HCSR04.d \
./Core/HCSR04/HCSR04_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HCSR04/%.o Core/HCSR04/%.su Core/HCSR04/%.cyclo: ../Core/HCSR04/%.c Core/HCSR04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HCSR04

clean-Core-2f-HCSR04:
	-$(RM) ./Core/HCSR04/HCSR04.cyclo ./Core/HCSR04/HCSR04.d ./Core/HCSR04/HCSR04.o ./Core/HCSR04/HCSR04.su ./Core/HCSR04/HCSR04_cfg.cyclo ./Core/HCSR04/HCSR04_cfg.d ./Core/HCSR04/HCSR04_cfg.o ./Core/HCSR04/HCSR04_cfg.su

.PHONY: clean-Core-2f-HCSR04

