################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f401retx.s 

OBJS += \
./Core/Startup/startup_stm32f401retx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221/ros" -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221/ros -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221 -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Core/Inc/rosSerial -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

