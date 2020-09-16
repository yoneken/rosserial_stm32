# chatter (an example for rosserial_stm32)
The *rosserial_stm32 chatter example* shows how to publish a string message over USART2 through DMA.

## Prerequisites

The example depends on folllowing packages and tools:
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) or [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)
* [ROS](https://www.ros.org/)
* [rosserial](http://wiki.ros.org/rosserial)
* [rosserial_stm32](https://github.com/yoneken/rosserial_stm32)

## Installation guide

To run this example, install prerequired dependencies.

### ROS installation

Please, report to [ROS installation page](http://wiki.ros.org/melodic/Installation/Ubuntu)

### STM32CubeIDE installation

* Download [STM32CubeIDE from ST website](https://www.st.com/en/development-tools/stm32cubeide.html)

* From your download repository, open up a console and run below command to unzip the downloaded file

```
unzip {FILE_NAME.zip}
```

* Run the bash script with execution priviledge and follow installation instuctions.

```
chmod +x {FILE_NAME.sh}
sh {FILE_NAME.sh}
```

***Note: Press 'Q' character to skip software compliance***

### ROSserial installation

From your console, run the following commands:


```
sudo apt update
sudo apt-get install ros-melodic-rosserial
```
**Note: Replace _melodic_ by your ROS distribution**

### ROSserial_stm32 installation

* Clone [rosserial_stm32](https://github.com/yoneken/rosserial_stm32) repository:
```
git clone https://github.com/yoneken/rosserial_stm32
```

* Build the project
```
make install
```

* Copy package path folder to your ROS setup
```
echo "[ABSOLUT_PATH_TO_ROSSERIAL_STM32_FOLDER]/rosserial_stm32/src/rosserial_stm32" >> ~/.bashrc
source ~/.bashrc
```

## Target your board

The example has been tested on following Nucleo kits.

- [Nucleo-F303K8(STM32F303)](http://www.st.com/en/evaluation-tools/nucleo-f303k8.html)
- [Nucleo-F446RE(STM32F446)](http://www.st.com/en/evaluation-tools/nucleo-f446re.html)
- [Nucleo-F401RE(STM32F401RE)](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-f401re.html)

To target your development kit rename the corresponding _.ioc_ file to _chatter.ioc_ __and__ change [the device definition](https://github.com/yoneken/rosserial_stm32/blob/master/src/ros_lib/STM32Hardware.h#L38).

For example, when you use Nucleo-F446RE, overwrite *chatter_F446RE.ioc* to _chatter.ioc_ and change [the device definition](https://github.com/yoneken/rosserial_stm32/blob/master/src/ros_lib/STM32Hardware.h#L38).

If your kit is not present within the list, you might need to edit the _ioc_ file or import source files in a new project.

## Run the example

* Once your board is connected and programmed, you need to get the serial port using *dmesg* command


```
dmesg | grep tty
```

* Launch ROS core server

```
roscore
```

* Launch the rosserial service :

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
*Note: Replace 'ttyACM0' by the connected serial port*

* Listen the chatter topic :

```
rostopic echo chatter
```