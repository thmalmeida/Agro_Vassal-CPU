################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Sys/subdir.mk
-include RTC/subdir.mk
-include Comm/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Agro_Vassal-CPU.lss \

FLASH_IMAGE += \
Agro_Vassal-CPU.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: Agro_Vassal-CPU.elf secondary-outputs

# Tool invocations
Agro_Vassal-CPU.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,Agro_Vassal-CPU.map,--cref -mrelax -Wl,--gc-sections -Wl,--gc-sections -L"C:\Users\Thiago\sourceCodes1\arduino_core\328P_16MHz" -mmcu=atmega328p -o "Agro_Vassal-CPU.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Agro_Vassal-CPU.lss: Agro_Vassal-CPU.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Agro_Vassal-CPU.elf  >"Agro_Vassal-CPU.lss"
	@echo 'Finished building: $@'
	@echo ' '

Agro_Vassal-CPU.hex: Agro_Vassal-CPU.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Agro_Vassal-CPU.elf  "Agro_Vassal-CPU.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Agro_Vassal-CPU.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Agro_Vassal-CPU.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) Agro_Vassal-CPU.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
