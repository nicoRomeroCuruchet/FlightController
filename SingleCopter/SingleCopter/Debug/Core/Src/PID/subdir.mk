################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/PID/PID.c 

OBJS += \
./Core/Src/PID/PID.o 

C_DEPS += \
./Core/Src/PID/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/PID/%.o Core/Src/PID/%.su Core/Src/PID/%.cyclo: ../Core/Src/PID/%.c Core/Src/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-PID

clean-Core-2f-Src-2f-PID:
	-$(RM) ./Core/Src/PID/PID.cyclo ./Core/Src/PID/PID.d ./Core/Src/PID/PID.o ./Core/Src/PID/PID.su

.PHONY: clean-Core-2f-Src-2f-PID
