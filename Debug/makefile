################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include stm_startup/subdir.mk
-include src/soft_timers/subdir.mk
-include src/common/subdir.mk
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
all: SPI_stm32f10x.elf

# Tool invocations
SPI_stm32f10x.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	arm-none-eabi-gcc -L"/home/kripton/Applications/ARM_workspace/SPI_stm32f10x/lib" -T "/home/kripton/Applications/ARM_workspace/SPI_stm32f10x/stm32f103.ld" -g3  -ggdb -O0 -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd   -Wl,-Map=linker.map -Wl,-cref  -Wl,--gc-sections -o "SPI_stm32f10x.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) SPI_stm32f10x.elf
	-@echo ' '

post-build:
	-@echo 'Create binary'
	-arm-none-eabi-objcopy -S  -O binary  "SPI_stm32f10x.elf" "SPI_stm32f10x.bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
