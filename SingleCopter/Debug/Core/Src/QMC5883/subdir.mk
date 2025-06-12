################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/QMC5883/QMC5883.c 

OBJS += \
./Core/Src/QMC5883/QMC5883.o 

C_DEPS += \
./Core/Src/QMC5883/QMC5883.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/QMC5883/%.o Core/Src/QMC5883/%.su Core/Src/QMC5883/%.cyclo: ../Core/Src/QMC5883/%.c Core/Src/QMC5883/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-QMC5883

clean-Core-2f-Src-2f-QMC5883:
	-$(RM) ./Core/Src/QMC5883/QMC5883.cyclo ./Core/Src/QMC5883/QMC5883.d ./Core/Src/QMC5883/QMC5883.o ./Core/Src/QMC5883/QMC5883.su

.PHONY: clean-Core-2f-Src-2f-QMC5883

