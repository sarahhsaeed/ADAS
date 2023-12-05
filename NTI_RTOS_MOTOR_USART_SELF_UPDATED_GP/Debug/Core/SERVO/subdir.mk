################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/SERVO/SERVO.c \
../Core/SERVO/SERVO_cfg.c 

OBJS += \
./Core/SERVO/SERVO.o \
./Core/SERVO/SERVO_cfg.o 

C_DEPS += \
./Core/SERVO/SERVO.d \
./Core/SERVO/SERVO_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/SERVO/%.o Core/SERVO/%.su Core/SERVO/%.cyclo: ../Core/SERVO/%.c Core/SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-SERVO

clean-Core-2f-SERVO:
	-$(RM) ./Core/SERVO/SERVO.cyclo ./Core/SERVO/SERVO.d ./Core/SERVO/SERVO.o ./Core/SERVO/SERVO.su ./Core/SERVO/SERVO_cfg.cyclo ./Core/SERVO/SERVO_cfg.d ./Core/SERVO/SERVO_cfg.o ./Core/SERVO/SERVO_cfg.su

.PHONY: clean-Core-2f-SERVO

