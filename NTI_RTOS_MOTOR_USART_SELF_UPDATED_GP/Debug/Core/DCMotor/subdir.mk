################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/DCMotor/DCMotor.c 

OBJS += \
./Core/DCMotor/DCMotor.o 

C_DEPS += \
./Core/DCMotor/DCMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/DCMotor/%.o Core/DCMotor/%.su Core/DCMotor/%.cyclo: ../Core/DCMotor/%.c Core/DCMotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-DCMotor

clean-Core-2f-DCMotor:
	-$(RM) ./Core/DCMotor/DCMotor.cyclo ./Core/DCMotor/DCMotor.d ./Core/DCMotor/DCMotor.o ./Core/DCMotor/DCMotor.su

.PHONY: clean-Core-2f-DCMotor

