# Thermocamera (an example for rosserial_stm32)

## HAL
- [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)

## Target board
- [Nucleo-F303K8(STM32F303)](http://www.st.com/en/evaluation-tools/nucleo-f303k8.html)

## Device
- [Adafruit AMG8833 IR Thermal Camera Breakout](https://www.adafruit.com/product/3538)

## Using Peripherals
- USART2 (through DMA)
- Timer2
- I2C1 (through DMA)

## Wire
|Name    |Nucreo board|STM32F303(LQFP32)|Adafruit board|AMG8833        |
|--------|------------|-----------------|--------------|---------------|
|Vin (5V)|CN4-P4      |                 |VIN           |VDD(9), VPP(13)|
|GND     |CN4-P2      |VSS(16, 32)      |GND           |GND(6)         |
|SCL     |CN3-P8      |PB6(29)          |SCL           |SCL(3)         |
|SDA     |CN3-P7      |PB7(30)          |SDA           |SDA(2)         |
