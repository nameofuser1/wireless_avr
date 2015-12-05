################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include stm_startup/subdir.mk
-include src/subdir.mk
-include cmsis_core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: STM32F10_TEMPLATE

# Tool invocations
STM32F10_TEMPLATE: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	arm-none-eabi-gcc -L/home/kripton/Applications/EmbeddedArm/gcc-arm-none-eabi-4_9-2015q2/arm-none-eabi/lib -T "/home/kripton/Applications/ARM_workspace/STM32F10_TEMPLATE/stm32f103.ld" -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd   -Wl,-Map=linker.map -Wl,-cref  -Wl,--gc-sections -o "STM32F10_TEMPLATE" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) STM32F10_TEMPLATE
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
