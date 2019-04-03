# LED (an example for rosserial_stm32)
The basic example for rosserial_stm32

## HAL
- [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)

## Target board
- [Nucleo-F303K8(STM32F303)](http://www.st.com/en/evaluation-tools/nucleo-f303k8.html)

## Using Peripherals
- USART2 (through DMA)
- Timer1
- Timer2

## Wire
|Name    |Nucreo board|STM32F303(LQFP32)|LED  |
|--------|------------|-----------------|-----|
|Vin (5V)|CN4-P4      |                 |     |
|GND     |CN4-P2      |VSS(16, 32)      |GND  |
|LED0    |CN3-P12     |PA8(18)          |LED0 |
|LED1    |CN3-P1      |PA9(19)          |LED1 |
|LED2    |CN3-P2      |PA10(20)         |LED2 |
|LED3    |CN3-P13     |PA11(21)         |LED3 |

## Note
This code requires a bit big transmit buffer.
You have to enlarge the buffer in STM32FHardware.h like this.

```
  const static uint16_t tbuflen = 512;
```

