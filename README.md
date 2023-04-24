# GD32-hover-master-slave

Based on Hoverboard-Firmware-Hack-Gen2 https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
and https://github.com/gaucho1978/CHEAP-LAWNMOWER-ROBOT-FROM-HOVERBOARD

This repository is for Makefile only. I added Makefile, Linker, startup, and Drivers for GD32.

- Makefile is ready
	
	type "make" to compile
	
	type "make flash" to compile and flash

#### Hardware

The reverse-engineered schematics of the mainboards are below.

The hardware has two main boards, which are different equipped. They are connected via USART. Additionally there are some LED PCB connected at X1 and X2 which signalize the battery state and the error state. There is an programming connector for ST-Link/V2 and they break out GND, USART/I2C, 5V on a second pinhead.

![MasterSlave](https://github.com/weiminshen99/GD32-hover-master-slave/blob/main/Docs/Hardware_Overview_small.png)

![Schematic](https://github.com/weiminshen99/GD32-hover-master-slave/blob/main/Docs/Schematic.pdf)
