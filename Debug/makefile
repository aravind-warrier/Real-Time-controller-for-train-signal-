################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/FreeRTOS/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Real-Time Railway Crossing Controller Simulation
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Real-Time\ Railway\ Crossing\ Controller\ Simulation.elf \

MAP_FILES += \
Real-Time\ Railway\ Crossing\ Controller\ Simulation.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Real-Time\ Railway\ Crossing\ Controller\ Simulation.list \

OBJCOPY_HEX += \
Real-Time\ Railway\ Crossing\ Controller\ Simulation.hex \

OBJCOPY_BIN += \
Real-Time\ Railway\ Crossing\ Controller\ Simulation.bin \


# All Target
all: main-build

# Main-build Target
main-build: Real-Time\ Railway\ Crossing\ Controller\ Simulation.elf secondary-outputs

# Tool invocations
Real-Time\ Railway\ Crossing\ Controller\ Simulation.elf Real-Time\ Railway\ Crossing\ Controller\ Simulation.map: $(OBJS) $(USER_OBJS) C:\Users\aravi\STM32CubeIDE\workspace_1.18.0\Real-Time\ Railway\ Crossing\ Controller\ Simulation\STM32F446VETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Real-Time Railway Crossing Controller Simulation.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\aravi\STM32CubeIDE\workspace_1.18.0\Real-Time Railway Crossing Controller Simulation\STM32F446VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Real-Time Railway Crossing Controller Simulation.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Real-Time\ Railway\ Crossing\ Controller\ Simulation.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Real-Time Railway Crossing Controller Simulation.list"
	@echo 'Finished building: $@'
	@echo ' '

Real-Time\ Railway\ Crossing\ Controller\ Simulation.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Real-Time Railway Crossing Controller Simulation.hex"
	@echo 'Finished building: $@'
	@echo ' '

Real-Time\ Railway\ Crossing\ Controller\ Simulation.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Real-Time Railway Crossing Controller Simulation.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Real-Time\ Railway\ Crossing\ Controller\ Simulation.bin Real-Time\ Railway\ Crossing\ Controller\ Simulation.elf Real-Time\ Railway\ Crossing\ Controller\ Simulation.hex Real-Time\ Railway\ Crossing\ Controller\ Simulation.list Real-Time\ Railway\ Crossing\ Controller\ Simulation.map Real-Time\\ Railway\\ Crossing\\ Controller\\ Simulation.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
