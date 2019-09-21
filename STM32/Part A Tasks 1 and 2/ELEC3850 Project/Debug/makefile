################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
ELEC3850\ Project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ELEC3850\ Project.list \


# All Target
all: ELEC3850\ Project.elf secondary-outputs

# Tool invocations
ELEC3850\ Project.elf: $(OBJS) $(USER_OBJS) C:\Users\Gordon\Documents\UON\Year\ 3\Semester\ 2\ELEC3850\ELEC3850\ Project\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "ELEC3850 Project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Gordon\Documents\UON\Year 3\Semester 2\ELEC3850\ELEC3850 Project\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ELEC3850 Project.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ELEC3850\ Project.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ELEC3850 Project.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
