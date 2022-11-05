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

void ShowBatteryState(uint32_t pin);

void loop_delay(uint16_t ms)
{
        volatile int count = 1000 * ms;
        while ( count-- );
}

//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
int main (void)
{

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 100);

  // Init GPIOs
  GPIO_init();

  while(1) {
	ShowBatteryState(LED_GREEN);
	loop_delay(1000);
	ShowBatteryState(LED_ORANGE);
	loop_delay(1000);
	ShowBatteryState(LED_RED);
	loop_delay(1000);
  }
}

//----------------------------------------------------------------------------
// Shows the battery state on the LEDs
//----------------------------------------------------------------------------
void ShowBatteryState(uint32_t pin)
{
	gpio_bit_write(LED_GREEN_PORT, LED_GREEN, pin == LED_GREEN ? SET : RESET);
	gpio_bit_write(LED_ORANGE_PORT, LED_ORANGE, pin == LED_ORANGE ? SET : RESET);
	gpio_bit_write(LED_RED_PORT, LED_RED, pin == LED_RED ? SET : RESET);
}

