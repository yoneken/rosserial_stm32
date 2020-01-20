# chatter (an example for rosserial_stm32)
The basic example for rosserial_stm32

## HAL
- [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)

## Target board
- [Nucleo-F303K8(STM32F303)](http://www.st.com/en/evaluation-tools/nucleo-f303k8.html)
- [Nucleo-F446RE(STM32F446)](http://www.st.com/en/evaluation-tools/nucleo-f446re.html)

When you use F446RE, overwrite chatter_f4.ioc to chatter.ioc and change [the device definition](https://github.com/yoneken/rosserial_stm32/blob/master/src/ros_lib/STM32Hardware.h#L38).

## Using Peripherals
- USART2 (through DMA)