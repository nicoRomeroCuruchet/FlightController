################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Quaternion/quaternion_utils.c 

OBJS += \
./Core/Src/Quaternion/quaternion_utils.o 

C_DEPS += \
./Core/Src/Quaternion/quaternion_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Quaternion/%.o Core/Src/Quaternion/%.su Core/Src/Quaternion/%.cyclo: ../Core/Src/Quaternion/%.c Core/Src/Quaternion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Quaternion

clean-Core-2f-Src-2f-Quaternion:
	-$(RM) ./Core/Src/Quaternion/quaternion_utils.cyclo ./Core/Src/Quaternion/quaternion_utils.d ./Core/Src/Quaternion/quaternion_utils.o ./Core/Src/Quaternion/quaternion_utils.su

.PHONY: clean-Core-2f-Src-2f-Quaternion

