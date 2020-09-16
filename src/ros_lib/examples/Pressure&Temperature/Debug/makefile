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
-include Core/Inc/subdir.mk
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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
X_NUCLEO_IKS01A2_Shield.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
X_NUCLEO_IKS01A2_Shield.list \

OBJCOPY_BIN += \
X_NUCLEO_IKS01A2_Shield.bin \


# All Target
all: X_NUCLEO_IKS01A2_Shield.elf secondary-outputs

# Tool invocations
X_NUCLEO_IKS01A2_Shield.elf: $(OBJS) $(USER_OBJS) /home/fofolevrai/Documents/robotic/STm_ws/Training/X_NUCLEO_IKS01A2_Shield/STM32F401RETX_FLASH.ld
	arm-none-eabi-g++ -o "X_NUCLEO_IKS01A2_Shield.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/fofolevrai/Documents/robotic/STm_ws/Training/X_NUCLEO_IKS01A2_Shield/STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="X_NUCLEO_IKS01A2_Shield.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

X_NUCLEO_IKS01A2_Shield.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "X_NUCLEO_IKS01A2_Shield.list"
	@echo 'Finished building: $@'
	@echo ' '

X_NUCLEO_IKS01A2_Shield.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "X_NUCLEO_IKS01A2_Shield.bin"
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
