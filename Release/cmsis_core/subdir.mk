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
	arm-none-eabi-gcc -DSTM32F10X_MD -I/home/kripton/Applications/EmbeddedArm/gcc-arm-none-eabi-4_9-2015q2/arm-none-eabi/include -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_inc -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/Driver -O3 -Wall -c -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


