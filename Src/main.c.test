#define ARM_MATH_CM3

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include "gd32f1x0.h"
#include "gd32f1x0_gpio.h"

#include "../Inc/setup.h"
#include "../Inc/defines.h"
#include "../Inc/config.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include "../Inc/commsMasterSlave.h"
#include "../Inc/commsSteering.h"
#include "../Inc/commsBluetooth.h"


void loop_delay(uint16_t ms)
{
        volatile int count = 1000 * ms;
        while ( count-- );

	//extern uint32_t msTicks;
	//uint16_t end_ms = msTicks + ms;
	//while (msTicks < end_ms);
}


void ShowBatteryState(uint32_t pin)
{
	gpio_bit_write(LED_GREEN_PORT, LED_GREEN, pin == LED_GREEN ? SET : RESET);
	gpio_bit_write(LED_ORANGE_PORT, LED_ORANGE, pin == LED_ORANGE ? SET : RESET);
	gpio_bit_write(LED_RED_PORT, LED_RED, pin == LED_RED ? SET : RESET);
}


//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
int main (void)
{

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
        Interrupt_init();

        // Init timeout timer
        TimeoutTimer_init();

        // Init GPIOs
        GPIO_init();

        // Activate self hold direct after GPIO-init
        gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET);

        // Init usart master slave
        USART_MasterSlave_init();

        // Init ADC
        ADC_init();

        // Init PWM
        PWM_init();

        // Device has 1,6 seconds to do all the initialization
        // afterwards watchdog will be fired
//        fwdgt_counter_reload();

	SetEnable(SET);
	SetPWM(250);

  while(1) {

	ShowBatteryState(LED_ORANGE);
	loop_delay(100);
	ShowBatteryState(LED_RED);
	loop_delay(100);
	ShowBatteryState(LED_GREEN);
	loop_delay(100);

	//gpio_bit_write(LED_GREEN_PORT, LED_GREEN, SET);
	//loop_delay(100);
        //gpio_bit_write(LED_GREEN_PORT, LED_GREEN, RESET);
	//loop_delay(100);

	// fwdgt_counter_reload();
  }
}

