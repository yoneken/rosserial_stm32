# Pressure sensor (an example for rosserial_stm32)

## HAL
- [STM32CubeIDE](http://www.st.com/en/development-tools/stm32cubeide.html)

## Target board
- [Nucleo-F303K8(STM32F303)](http://www.st.com/en/evaluation-tools/nucleo-f303k8.html)

## Device
- [MIS-2500-015V](http://www.metrodyne.com.tw/english/product/prodlist/mis-2500.htm)

## Using Peripherals
- USART2 (through DMA)
- ADC1 (through DMA)
- Timer6 (trigger ADC1)

## Wire
|Name      |Nucreo board|STM32F303(LQFP32)|MIS-2500  |
|----------|------------|-----------------|----------|
|Vin (3.3V)|CN4-P14     |                 |Vsupply   |
|GND       |CN4-P2      |VSS(16, 32)      |Ground    |
|ADC1_IN1  |CN4-P5      |PA2(8)           |Output    |

