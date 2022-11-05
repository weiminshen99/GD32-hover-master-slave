/*
* This file is part of the hoverboard-firmware-hack-V2 project. The
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The
* structure was tried to be as similar as possible, so that everyone
* could find a better way through the code.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define ARM_MATH_CM3

#include "gd32f1x0.h"

#include "../Inc/setup.h"
#include "../Inc/defines.h"
#include "../Inc/config.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include "../Inc/commsMasterSlave.h"
#include "../Inc/commsSteering.h"
#include "../Inc/commsBluetooth.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
//#include "arm_math.h"


//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
int main (void)
{
	//SystemClock_Config();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 100);

/*
	// Init watchdog
	if (Watchdog_init() == ERROR)
	{// If an error accours with watchdog initialization do not start device
		while(1);
	}
*/
	// Init Interrupts
//	Interrupt_init();

	// Init timeout timer
//	TimeoutTimer_init();

	// Init GPIOs
	GPIO_init();

	// Activate self hold direct after GPIO-init
//	gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET);

	// Init usart master slave
//	USART_MasterSlave_init();

	// Init ADC
//	ADC_init();

	// Init PWM
//	PWM_init();

	// Device has 1,6 seconds to do all the initialization
	// afterwards watchdog will be fired
	//fwdgt_counter_reload();

	// Init usart steer/bluetooth
	//USART_Steer_COM_init();

	volatile uint32_t main_loop_counter = 0;

	gpio_bit_write(LED_GREEN_PORT, LED_GREEN, RESET);

  	while(1) // MAIN LOOP
    	{
		main_loop_counter++;

		if (main_loop_counter < 1000) {
//	  		gpio_bit_write(LED_GREEN_PORT, LED_GREEN, SET);
	  		//gpio_bit_write(LED_RED_PORT, LED_RED, RESET);
		} else if (main_loop_counter < 2000) {
	  		//gpio_bit_write(LED_GREEN_PORT, LED_GREEN, RESET);
	  		gpio_bit_write(LED_RED_PORT, LED_RED, SET);
		} else {
	  		main_loop_counter = 0;
//	  		gpio_bit_write(LED_GREEN_PORT, LED_GREEN, RESET);
	  		gpio_bit_write(LED_RED_PORT, LED_RED, RESET);
		}

		//SetEnable(SET);	// enable motor on SLAVE
		//SetPWM(250);	// testing BLDC on SLAVE

//		Delay(DELAY_IN_MAIN_LOOP);

//		fwdgt_counter_reload();

	}
}
