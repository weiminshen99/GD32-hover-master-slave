//configuration file
//all general settings should be defined here
//in example, you should define MASTER or SLAVE in order to compile firmware for master or slave board.
//don't touch other things if you don't know what you're doing

#ifndef CONFIG_H
	#define CONFIG_H

	#include "gd32f1x0.h"

	// ################################################################################
	//#define MASTER	// Select if firmware is for master or slave board
	#define SLAVE 		// Select if firmware is for master or slave board
	// ################################################################################
	#define PWM_FREQ         		16000     // PWM frequency in Hz
	#define DEAD_TIME        		60        // PWM deadtime (60 = 1�s, measured by oscilloscope)
	#define DC_CUR_LIMIT     		15        // Motor DC current limit in amps
	// ################################################################################
	#define DELAY_IN_MAIN_LOOP 	5         // Delay in ms
	#define TIMEOUT_MS          2000      // Time in milliseconds without steering commands before pwm emergency off
	// ################################################################################
	#ifdef MASTER
		#define INACTIVITY_TIMEOUT 	8        	// Minutes of not driving until poweroff (not very precise)
		// ################################################################################
		#define BAT_LOW_LVL1     35.0       // Gently beeps, show green battery symbol above this Level.
		#define BAT_LOW_LVL2     33.0       // Battery almost empty, show orange battery symbol above this Level. Charge now! 
		#define BAT_LOW_DEAD     31.0       // Undervoltage lockout, show red battery symbol above this Level.
		// ONLY DEBUG-LEVEL!!!
		//#define BAT_LOW_LVL1     29.0
		//#define BAT_LOW_LVL2     28.0
 		//#define BAT_LOW_DEAD     27.0
		// ################################################################################
	#endif

	// ###### ARMCHAIR ######
	#define FILTER_SHIFT 12 						// Low-pass filter for pwm, rank k=12
	#define FILTER_SHIFT2 4 						// Low-pass filter for pwm, rank k=12
	#define CONFIRMATIONSCOUNT 10				// low pass filter for pwm signal coming from radiocontrol
	#ifdef MASTER
		#define SPEED_COEFFICIENT   -1
		#define STEER_COEFFICIENT   1
		#define REMOTE_CONTROL_PWM          //comment this line if pwm Radio CONTROL of the lawnmower is not connected
		#define DEBUG_ENABLED
		#define TERMINAL_ENABLED 					// uncomment this line to connect a terminal on the remote port and use it as debugging terminal port. this will generate a firmware bigger than 32kB.
		//#define TERMINAL_ENABLED_PID_TUNING // this is like TERMINAL_ENABLED but it allows just pid tuning of the slave wheel, via serial port of master board
		#ifndef TERMINAL_ENABLED
			#ifndef TERMINAL_ENABLED_PID_TUNING
				//#define MAVLINK_ENABLED //enable gps decoding on remote port.
			#endif
		#endif
#endif

	#define DEBUG_WITH_TRACE_ENABLED //if enabled PB3 is left free for trace purpose. with keil you can use trace to see variables in a graph
	#define DISTANCE_BETWEEN_WHEELS 520.0f	//millimeters era 525
	#define WHEEL_PERIMETER 542.0f //millimeters
#endif
