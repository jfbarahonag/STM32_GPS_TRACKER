################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
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
GPS_TRACKER.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GPS_TRACKER.list \

OBJCOPY_BIN += \
GPS_TRACKER.bin \


# All Target
all: GPS_TRACKER.elf secondary-outputs

# Tool invocations
GPS_TRACKER.elf: $(OBJS) $(USER_OBJS) D:\Workspace\GPS_TRACKER\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "GPS_TRACKER.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Workspace\GPS_TRACKER\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GPS_TRACKER.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GPS_TRACKER.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GPS_TRACKER.list"
	@echo 'Finished building: $@'
	@echo ' '

GPS_TRACKER.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "GPS_TRACKER.bin"
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
