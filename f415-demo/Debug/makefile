################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
f415-demo.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
f415-demo.list \

OBJCOPY_BIN += \
f415-demo.bin \


# All Target
all: f415-demo.elf secondary-outputs

# Tool invocations
f415-demo.elf: $(OBJS) $(USER_OBJS) C:\Users\baser\STM32CubeIDE\workspace_1.3.0\f415-demo\STM32F415RGTX_FLASH.ld
	arm-none-eabi-gcc -o "f415-demo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\baser\STM32CubeIDE\workspace_1.3.0\f415-demo\STM32F415RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="f415-demo.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

f415-demo.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "f415-demo.list"
	@echo 'Finished building: $@'
	@echo ' '

f415-demo.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "f415-demo.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
