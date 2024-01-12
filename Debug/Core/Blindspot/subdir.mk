################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Blindspot/blindspot_prog.c 

OBJS += \
./Core/Blindspot/blindspot_prog.o 

C_DEPS += \
./Core/Blindspot/blindspot_prog.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Blindspot/%.o Core/Blindspot/%.su Core/Blindspot/%.cyclo: ../Core/Blindspot/%.c Core/Blindspot/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Blindspot

clean-Core-2f-Blindspot:
	-$(RM) ./Core/Blindspot/blindspot_prog.cyclo ./Core/Blindspot/blindspot_prog.d ./Core/Blindspot/blindspot_prog.o ./Core/Blindspot/blindspot_prog.su

.PHONY: clean-Core-2f-Blindspot

