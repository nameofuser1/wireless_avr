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
	arm-none-eabi-as -mthumb -mcpu=cortex-m3 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm_startup/%.o: ../stm_startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DSTM32F10X_MD -I/home/kripton/Applications/EmbeddedArm/gcc-arm-none-eabi-4_9-2015q2/arm-none-eabi/include -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_inc -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/Driver -O3 -Wall -c -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


