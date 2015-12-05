################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm_startup/system_stm32f10x.c 

S_UPPER_SRCS += \
../stm_startup/startup_stm32f10x_md.S 

OBJS += \
./stm_startup/startup_stm32f10x_md.o \
./stm_startup/system_stm32f10x.o 

C_DEPS += \
./stm_startup/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
stm_startup/%.o: ../stm_startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	arm-none-eabi-as -mthumb -mcpu=cortex-m3 -g3 -ggdb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm_startup/%.o: ../stm_startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I/home/kripton/Applications/EmbeddedArm/newlib/include -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_inc -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/Driver -I/home/kripton/Applications/EmbeddedArm/STM32F10x_StdPeriph_Driver/inc -I"/home/kripton/Applications/ARM_workspace/SPI_stm32f10x/src" -O0 -g3 -Wall -c -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -g3 -O0 -ggdb -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


