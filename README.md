# GD32-hover-master-slave

This repository is based on Hoverboard-Firmware-Hack-Gen2.0 https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
and https://github.com/gaucho1978/CHEAP-LAWNMOWER-ROBOT-FROM-HOVERBOARD (in "Lawnmower" here). The Makefile here can also be used for other similar repositories, such as https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1 or 
https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x

To use the Makefile here, I added Makefile, Linker, startup, and Drivers for GD32.

#### Makefile is Ready
	
	type "cd ." or "cd Lawnmower"
	
	type "make" to compile 
	type "make GCC_PATH=xxx" if your gcc compiler is xxx

	type "make flash" to flash the compiled firmware to your board
	
#### Schematic

![Schematic](https://github.com/weiminshen99/GD32-hover-master-slave/blob/main/Docs/Schematic.pdf)

#### Hardware

The hardware has two main boards, which are different equipped. They are connected via USART. Additionally there are some LED PCB connected at X1 and X2 which signalize the battery state and the error state. There is an programming connector for ST-Link/V2 and they break out GND, USART/I2C, 5V on a second pinhead.

![MasterSlave](https://github.com/weiminshen99/GD32-hover-master-slave/blob/main/Docs/Hardware_Overview_small.png)
