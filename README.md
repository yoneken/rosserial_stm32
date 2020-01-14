# rosserial_stm32

## Note
This is a part of [rosserial](https://github.com/ros-drivers/rosserial) repository to communicate with ROS system through a USART for STM32 embedded system.

## Limitation
Currently, this code is focused on STM32F3xx series and it uses the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) or [STM32CubeMX HAL](http://www.st.com/en/development-tools/stm32cubemx.html).  
If you use the package for other series, please edit src/ros_lib/STM32Hardware.h .  
After that, we appreciate you sharing your code :)  

## Generate code
$ cd _target_workspace_ (It should contain Inc and Src directories).  
$ rosrun rosserial_stm32 make_libraries.py .  
**Never forget to change the project type to _cpp project_ in STM32CubeIDE or SW4STM32!!**  
**For STM32CubeIDE users:** [You have to add /Inc directory as a "Source Location" in the project properties(C/C++ General > Paths and Symbols > Source Location).](https://github.com/yoneken/rosserial_stm32/issues/10#issuecomment-573997253)  


## Examples
See src/ros_lib/examples  

## Share your implementation based on this repo here
_Make sure your code is working before adding your repo here ;)_

[fdila's repo, working with STM32CubeIDE and stm32f7 based board, CURRENTLY NOT MAINTAINED](https://github.com/fdila/rosserial_stm32f7)
