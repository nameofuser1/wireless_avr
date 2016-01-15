################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core/core_cm3.c 

OBJS += \
./cmsis_core/core_cm3.o 

C_DEPS += \
./cmsis_core/core_cm3.d 


# Each subdirectory must supply rules for building sources it contributes
cmsis_core/core_cm3.o: /home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core/core_cm3.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I/home/kripton/Applications/EmbeddedArm/newlib/include -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_inc -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/Driver -I/home/kripton/Applications/EmbeddedArm/STM32F10x_StdPeriph_Driver/inc -I"/home/kripton/Applications/ARM_workspace/SPI_stm32f10x/src" -O0 -g3 -Wall -Werror -c -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -g3 -O0 -ggdb -std=c99 -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


