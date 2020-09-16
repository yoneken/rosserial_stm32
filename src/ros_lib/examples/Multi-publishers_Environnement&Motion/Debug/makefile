################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MEMS/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/IKS01A2/subdir.mk
-include Drivers/BSP/Components/lsm6dsl/ros/subdir.mk
-include Drivers/BSP/Components/lsm6dsl/subdir.mk
-include Drivers/BSP/Components/lsm303agr/ros/subdir.mk
-include Drivers/BSP/Components/lsm303agr/subdir.mk
-include Drivers/BSP/Components/lps22hb/ros/subdir.mk
-include Drivers/BSP/Components/lps22hb/subdir.mk
-include Drivers/BSP/Components/hts221/ros/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/rosSerial/subdir.mk
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
ROS-X-NUCLEO-IKS01A2-EXAMPLE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ROS-X-NUCLEO-IKS01A2-EXAMPLE.list \

OBJCOPY_BIN += \
ROS-X-NUCLEO-IKS01A2-EXAMPLE.bin \


# All Target
all: ROS-X-NUCLEO-IKS01A2-EXAMPLE.elf secondary-outputs

# Tool invocations
ROS-X-NUCLEO-IKS01A2-EXAMPLE.elf: $(OBJS) $(USER_OBJS) /home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/STM32F401RETX_FLASH.ld
	arm-none-eabi-g++ -o "ROS-X-NUCLEO-IKS01A2-EXAMPLE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/STM32F401RETX_FLASH.ld" -Wl,-Map="ROS-X-NUCLEO-IKS01A2-EXAMPLE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ROS-X-NUCLEO-IKS01A2-EXAMPLE.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ROS-X-NUCLEO-IKS01A2-EXAMPLE.list"
	@echo 'Finished building: $@'
	@echo ' '

ROS-X-NUCLEO-IKS01A2-EXAMPLE.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ROS-X-NUCLEO-IKS01A2-EXAMPLE.bin"
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
