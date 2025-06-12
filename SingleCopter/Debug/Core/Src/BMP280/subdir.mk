################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMP280/BMP280.c 

OBJS += \
./Core/Src/BMP280/BMP280.o 

C_DEPS += \
./Core/Src/BMP280/BMP280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BMP280/%.o Core/Src/BMP280/%.su Core/Src/BMP280/%.cyclo: ../Core/Src/BMP280/%.c Core/Src/BMP280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BMP280

clean-Core-2f-Src-2f-BMP280:
	-$(RM) ./Core/Src/BMP280/BMP280.cyclo ./Core/Src/BMP280/BMP280.d ./Core/Src/BMP280/BMP280.o ./Core/Src/BMP280/BMP280.su

.PHONY: clean-Core-2f-Src-2f-BMP280

