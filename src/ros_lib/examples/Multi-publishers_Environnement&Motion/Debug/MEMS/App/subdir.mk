################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MEMS/App/app_x-cube-mems1.c 

OBJS += \
./MEMS/App/app_x-cube-mems1.o 

C_DEPS += \
./MEMS/App/app_x-cube-mems1.d 


# Each subdirectory must supply rules for building sources it contributes
MEMS/App/app_x-cube-mems1.o: ../MEMS/App/app_x-cube-mems1.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUSE_HAL_UART_REGISTER_CALLBACKS=1' -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../MEMS/Target -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../MEMS/App -I../Drivers/BSP/Components/lps22hb -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/lsm6dsl -I"/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221/ros" -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221/ros -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Drivers/BSP/Components/hts221 -I/home/fofolevrai/Documents/robotic/STm_ws/Training/en.x-cube-mems1/STM32CubeExpansion_MEMS1_V7.1.0/Projects/STM32F401RE-Nucleo/Examples/IKS01A2/ROS-X-NUCELO-IKS01A2/Core/Inc/rosSerial -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MEMS/App/app_x-cube-mems1.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

