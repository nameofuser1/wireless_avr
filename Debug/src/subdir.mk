################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/UART.c \
../src/avr_flasher.c \
../src/controller.c \
../src/flash_writer.c \
../src/main.c \
../src/spi.c \
../src/stm32f10x_gpio.c \
../src/stm32f10x_rcc.c \
../src/stm32f10x_usart.c \
../src/syscalls.c 

OBJS += \
./src/UART.o \
./src/avr_flasher.o \
./src/controller.o \
./src/flash_writer.o \
./src/main.o \
./src/spi.o \
./src/stm32f10x_gpio.o \
./src/stm32f10x_rcc.o \
./src/stm32f10x_usart.o \
./src/syscalls.o 

C_DEPS += \
./src/UART.d \
./src/avr_flasher.d \
./src/controller.d \
./src/flash_writer.d \
./src/main.d \
./src/spi.d \
./src/stm32f10x_gpio.d \
./src/stm32f10x_rcc.d \
./src/stm32f10x_usart.d \
./src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I/home/kripton/Applications/EmbeddedArm/newlib/include -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_core -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/cmsis_inc -I/home/kripton/Applications/EmbeddedArm/cmsis_stm32f10/Driver -I/home/kripton/Applications/EmbeddedArm/STM32F10x_StdPeriph_Driver/inc -I"/home/kripton/Applications/ARM_workspace/SPI_stm32f10x/src" -O0 -g3 -Wall -Werror -c -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -g3 -O0 -ggdb -std=c99 -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


