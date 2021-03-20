/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2017 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <asm/ioctl.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <poll.h>
#include <pthread.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "../include/wiringPi.h"

// Environment Variables

#define    ENV_DEBUG    "WIRINGPI_DEBUG"
#define    ENV_CODES    "WIRINGPI_CODES"
#define    ENV_GPIOMEM    "WIRINGPI_GPIOMEM"


// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure

struct wiringPiNodeStruct *wiringPiNodes = NULL;

// BCM Magic
#define    BCM_PASSWORD    0x5A000000


// The BCM2835 has 54 GPIO pins.
//	BCM2835 data sheet, Page 90 onwards.
//	There are 6 control registers, each control the functions of a block
//	of 10 pins.
//	Each control register has 10 sets of 3 bits per GPIO pin - the ALT values
//
//	000 = GPIO Pin X is an input
//	001 = GPIO Pin X is an output
//	100 = GPIO Pin X takes alternate function 0
//	101 = GPIO Pin X takes alternate function 1
//	110 = GPIO Pin X takes alternate function 2
//	111 = GPIO Pin X takes alternate function 3
//	011 = GPIO Pin X takes alternate function 4
//	010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//	X / 10 + ((X % 10) * 3)

// Port function select bits
#define    FSEL_ALT0    0b100
#define    FSEL_ALT5    0b010

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:
//
// Updates in September 2015 - all now static variables (and apologies for the caps)
//	due to the Pi v2, v3, etc. and the new /dev/gpiomem interface
static volatile unsigned int GPIO_PADS;
static volatile unsigned int GPIO_CLOCK_BASE;
static volatile unsigned int GPIO_BASE;
static volatile unsigned int GPIO_TIMER;
static volatile unsigned int GPIO_PWM;


// MAX_PINS:
//	This is more than the number of Pi pins because we can actually softPwm.
//	Once upon a time I let pins on gpio expanders be softPwm'd, but it's really
//	really not a good thing.
#define    MAX_PINS    64

#define    BLOCK_SIZE        (4*1024)

static unsigned int usingGpioMem = FALSE;
static int wiringPiSetuped = FALSE;

// PWM
//	Word offsets into the PWM control region
#define    PWM_CONTROL   0
#define    PWM0_RANGE    4
#define    PWM1_RANGE    8

//	Clock register offsets
#define    PWMCLK_CNTL   40
#define    PWMCLK_DIV    41

#define    PWM0_MS_MODE  0x0080  /**< Run in MS mode */
#define    PWM0_ENABLE   0x0001  /**< Channel Enable */

#define    PWM1_MS_MODE  0x8000  /**< Run in MS mode */
#define    PWM1_ENABLE   0x0100  /**< Channel Enable */

// Timer
//	Word offsets
#define    TIMER_CONTROL (0x408 >> 2)
#define    TIMER_IRQ_RAW (0x410 >> 2)
#define    TIMER_PRE_DIV (0x41C >> 2)

// Locals to hold pointers to the hardware
static volatile unsigned int *gpio;
static volatile unsigned int *pwm;
static volatile unsigned int *clk;
static volatile unsigned int *pads;
static volatile unsigned int *timer;
static volatile unsigned int *timerIrqRaw;

// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

// piGpioBase:
//	The base address of the GPIO memory mapped hardware IO
#define    GPIO_PERI_BASE_OLD  0x20000000
#define    GPIO_PERI_BASE_2835 0x3F000000
#define    GPIO_PERI_BASE_2711 0xFE000000

static volatile unsigned int piGpioBase = 0;

// Time for easy calculations
static uint64_t epochMilli, epochMicro;

// Misc
static int wiringPiMode = WPI_MODE_UNINITIALISED;

// Debugging & Return codes
int wiringPiDebug = FALSE;
int wiringPiReturnCodes = FALSE;

/**
 * @brief Map a file descriptor from the /sys/class/gpio/gpioX/value
 */
static int sysFds[64] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more inefficient than all the bit-twiddling, but it
//	does tend to make it all a bit clearer. At least to me!

/**
 * @brief Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
 * Cope for 3 different board revisions here.
 */
static int *pinToGpio;

// Revision 1, 1.1:
static int pinToGpioR1 [64] = {
	17, 18, 21, 22, 23, 24, 25, 4, // From the Original Wiki - GPIO 0 through 7: wpi  0 -  7
	0,  1,                         // I2C  - SDA1, SCL1		                     wpi  8 -  9
	8,  7,                         // SPI  - CE1, CE0		                     wpi 10 - 11
	10,  9, 11,                    // SPI  - MOSI, MISO, SCLK                    wpi 12 - 14
	14, 15,                        // UART - Tx, Rx			                     wpi 15 - 16
	// Padding:
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 31
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

// Revision 2:
static int pinToGpioR2[64] = {
	17,	18,	27,	22,	23,	24,	25,	4, // From the Original Wiki - GPIO 0 through 7: wpi  0 -  7
	2,	3,                         // I2C  - SDA0, SCL0			                 wpi  8 -  9
	8,	7,                         // SPI  - CE1, CE0			                 wpi 10 - 11
	10,	9, 	11,                    // SPI  - MOSI, MISO, SCLK	                 wpi 12 - 14
	14, 15,                        // UART - Tx, Rx				                 wpi 15 - 16
	28, 29, 30, 31,                // Rev 2: New GPIOs 8 though 11               wpi 17 - 20
	5, 6, 13, 19, 26,              // B+						                 wpi 21, 22, 23, 24, 25
	12, 16, 20, 21,                // B+						                 wpi 26, 27, 28, 29
	0, 1,                          // B+						                 wpi 30, 31
	// Padding:
	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,// ... 47
	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,// ... 63
};

/**
 * @brief Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin.
 * Cope for 2 different board revisions here.
 * Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56
 */
static int *physToGpio;

static int physToGpioR1 [64] = {
	    -1, // 0
	-1, -1, // 1 , 2
	0 , -1,
	1 , -1,
	4 , 14,
	-1, 15,
	17, 18,
	21, -1,
	22, 23,
	-1, 24,
	10, -1,
	9 , 25,
	11, 8 ,
	-1, 7 , // 25, 26
	                                            -1, -1, -1, -1, -1, // ... 31
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
};

static int physToGpioR2 [64] = {
		-1, // 0
	-1, -1, // 1 , 2
	2 , -1,
	3 , -1,
	4 , 14,
	-1, 15,
	17, 18,
	27, -1,
	22, 23,
	-1, 24,
	10, -1,
	9 , 25,
	11, 8 ,
	-1, 7 , // 25, 26
	// B+
	0,   1,
	5,  -1,
	6,  12,
	13, -1,
	19, 16,
	26, 20,
	-1, 21,
	// the P5 connector on the Rev 2 boards:
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	28, 29,
	30, 31,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
};

/**
 * @brief Map a BCM_GPIO pin to it's Function Selection control port.
 * (GPFSEL 0-5)
 * Groups of 10 - 3 bits per Function - 30 bits per port
 */
static uint8_t gpioToGPFSEL [] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

/**
 * @brief Define the shift up for the 3 bits per pin in each GPFSEL port
 */
static uint8_t gpioToShift [] = {
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
};

/**
 * @brief (Word) offset to the GPIO Set registers for each GPIO pin
 */
static uint8_t gpioToGPSET [] = {
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
};

/**
 * @brief (Word) offset to the GPIO Clear registers for each GPIO pin
 */
static uint8_t gpioToGPCLR [] = {
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
};

/**
 * @brief (Word) offset to the GPIO Input level registers for each GPIO pin
 */
static uint8_t gpioToGPLEV [] = {
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
};


#define	GPPUD	37   /**< GPIO Pin pull up/down register */
#define GPPUPPDN0 57 /**< Pin pull-up/down for pins 15:0  */
#define	GPIO_CLOCK_SOURCE	1

static volatile unsigned int piGpioPupOffset = 0;

/**
 * @brief (Word) offset to the Pull Up Down Clock register
 */
static uint8_t gpioToPUDCLK [] = {
	38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38,
	39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
};

/**
 * @brief the ALT value to put a GPIO pin into PWM mode
 */
static uint8_t gpioToPwmALT [] = {
	0        , 0        , 0        , 0        , 0        , 0        , 0, 0, //  0 ->  7
	0        , 0        , 0        , 0        , FSEL_ALT0, FSEL_ALT0, 0, 0, //  8 -> 15
	0        , 0        , FSEL_ALT5, FSEL_ALT5, 0        , 0        , 0, 0, // 16 -> 23
	0        , 0        , 0        , 0        , 0        , 0        , 0, 0, // 24 -> 31
	0        , 0        , 0        , 0        , 0        , 0        , 0, 0, // 32 -> 39
	FSEL_ALT0, FSEL_ALT0, 0        , 0        , 0        , FSEL_ALT0, 0, 0, // 40 -> 47
	0        , 0        , 0        , 0        , 0        , 0        , 0, 0, // 48 -> 55
	0        , 0        , 0        , 0        , 0        , 0        , 0, 0, // 56 -> 63
};



/**
 * @brief ALT value to put a GPIO pin into GP Clock mode.
 * On the Pi we can really only use BCM_GPIO_4 and BCM_GPIO_21
 * for clocks 0 and 1 respectively, however I'll include the full
 * list for completeness - maybe one day...
 */
static uint8_t gpioToGpClkALT0 [] = {
	0        , 0, 0        , 0        , FSEL_ALT0, FSEL_ALT0, FSEL_ALT0, 0, //  0 ->  7
	0        , 0, 0        , 0        , 0        , 0        , 0        , 0, //  8 -> 15
	0        , 0, 0        , 0        , FSEL_ALT5, FSEL_ALT5, 0        , 0, // 16 -> 23
	0        , 0, 0        , 0        , 0        , 0        , 0        , 0, // 24 -> 31
	FSEL_ALT0, 0, FSEL_ALT0, 0        , 0        , 0        , 0        , 0, // 32 -> 39
	0        , 0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0, 0        , 0        , 0, // 40 -> 47
	0        , 0, 0        , 0        , 0        , 0        , 0        , 0, // 48 -> 55
	0        , 0, 0        , 0        , 0        , 0        , 0        , 0, // 56 -> 63
};

/**
 * @brief (word) Offsets to the clock Control and Divisor register
 */
static uint8_t gpioToClkCon [] = {
	-1, -1, -1, -1, 28, 30, 32, -1, //  0 ->  7
	-1, -1, -1, -1, -1, -1, -1, -1, //  8 -> 15
	-1, -1, -1, -1, 28, 30, -1, -1, // 16 -> 23
	-1, -1, -1, -1, -1, -1, -1, -1, // 24 -> 31
	28, -1, 28, -1, -1, -1, -1, -1, // 32 -> 39
	-1, -1, 28, 30, 28, -1, -1, -1, // 40 -> 47
	-1, -1, -1, -1, -1, -1, -1, -1, // 48 -> 55
	-1, -1, -1, -1, -1, -1, -1, -1, // 56 -> 63
};

static uint8_t gpioToClkDiv [] = {
	-1, -1, -1, -1, 29, 31, 33, -1, //  0 ->  7
	-1, -1, -1, -1, -1, -1, -1, -1, //  8 -> 15
	-1, -1, -1, -1, 29, 31, -1, -1, // 16 -> 23
	-1, -1, -1, -1, -1, -1, -1, -1, // 24 -> 31
	29, -1, 29, -1, -1, -1, -1, -1, // 32 -> 39
	-1, -1, 29, 31, 29, -1, -1, -1, // 40 -> 47
	-1, -1, -1, -1, -1, -1, -1, -1, // 48 -> 55
	-1, -1, -1, -1, -1, -1, -1, -1, // 56 -> 63
};

// The PWM Frequency is derived from the "pulse time" below. Essentially,
//	the frequency is a function of the range and this pulse time.
//	The total period will be range * pulse time in µS, so a pulse time
//	of 100 and a range of 100 gives a period of 100 * 100 = 10,000 µS
//	which is a frequency of 100Hz.
//
//	It's possible to get a higher frequency by lowering the pulse time,
//	however CPU usage will skyrocket as wiringPi uses a hard-loop to time
//	periods under 100µS - this is because the Linux timer calls are just
//	not accurate at all, and have an overhead.
//
//	Another way to increase the frequency is to reduce the range - however
//	that reduces the overall output accuracy...
static int freqs[MAX_PINS];
static volatile int marks[MAX_PINS];
static volatile int range[MAX_PINS];
static volatile pthread_t threads[MAX_PINS];
static volatile int newPin = -1;

/**
 * @brief Thread to do the actual PWM output
 * @param arg
 * @return
 */
_Noreturn static void *softPwmThread(void *arg) {
	int pin, mark, space;
	struct sched_param param;

	param.sched_priority = sched_get_priority_max(SCHED_RR);
	pthread_setschedparam(pthread_self(), SCHED_RR, &param);

	pin = *((int *) arg);
	free(arg);

	pin = newPin;
	newPin = -1;

	piHiPri(90);

	for (;;) {
		mark = marks[pin];
		space = range[pin] - mark;

		if(mark != 0) {
			digitalWrite(pin, HIGH);
		}
		delayMicroseconds(mark * 100);

		if(space != 0) {
			digitalWrite(pin, LOW);
		}
		delayMicroseconds(space * 100);
	}

}

/**
 * @brief Create a new softPWM thread.
 * @param pin
 * @param initialValue
 * @param pwmRange
 * @return
 */
int softPwmCreate(int pin, int initialValue, int pwmRange) {
	int res;
	pthread_t myThread;
	int *passPin;

	if(pin >= MAX_PINS) {
		return -1;
	}

	if(range[pin] != 0) {    // Already running on this pin
		return -1;
	}

	if(pwmRange <= 0) {
		return -1;
	}

	passPin = malloc(sizeof(*passPin));
	if(passPin == NULL) {
		return -1;
	}

	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);

	marks[pin] = initialValue;
	range[pin] = pwmRange;

	*passPin = pin;
	newPin = pin;
	res = pthread_create(&myThread, NULL, softPwmThread, (void *) passPin);

	while (newPin != -1) {
		delay(1);
	}

	threads[pin] = myThread;

	return res;
}

/**
 * @brief Stop an existing softPWM thread
 * @param pin
 */
void softPwmStop(int pin) {
	if(pin < MAX_PINS) {
		if(range[pin] != 0) {
			pthread_cancel(threads[pin]);
			pthread_join(threads[pin], NULL);
			range[pin] = 0;
			digitalWrite(pin, LOW);
		}
	}
}

/**
 * @brief Thread to do the actual PWM output
 * @param dummy
 * @return
 */
_Noreturn static PI_THREAD (softToneThread) {
	int pin, freq, halfPeriod;
	struct sched_param param;

	param.sched_priority = sched_get_priority_max(SCHED_RR);
	pthread_setschedparam(pthread_self(), SCHED_RR, &param);

	pin = newPin;
	newPin = -1;

	piHiPri(50);

	for (;;) {
		freq = freqs[pin];
		if(freq == 0) {
			delay(1);
		} else {
			halfPeriod = 500000 / freq;

			digitalWrite(pin, HIGH);
			delayMicroseconds(halfPeriod);

			digitalWrite(pin, LOW);
			delayMicroseconds(halfPeriod);
		}
	}
}

/**
 * @brief Create a new tone thread.
 * @param pin
 * @return
 */
int softToneCreate(int pin) {
	int res;
	pthread_t myThread;

	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);

	if(threads[pin] != 0) {
		return -1;
	}

	freqs[pin] = 0;

	newPin = pin;
	res = pthread_create(&myThread, NULL, softToneThread, NULL);

	while (newPin != -1) {
		delay(1);
	}

	threads[pin] = myThread;

	return res;
}

/**
 * @brief Stop an existing softTone thread
 * @param pin
 */
void softToneStop(int pin) {
	if(threads[pin] != 0) {
		pthread_cancel(threads[pin]);
		pthread_join(threads[pin], NULL);
		threads[pin] = 0;
		digitalWrite(pin, LOW);
	}
}


/*!*********************************************************************************
 * Functions
 **********************************************************************************/

/**
 * @brief Attempt to set a high priority scheduling for the running program
 * @param pri
 * @return
 */
int piHiPri(const int pri) {
	struct sched_param sched;

	memset(&sched, 0, sizeof(sched));

	if(pri > sched_get_priority_max(SCHED_RR)) {
		sched.sched_priority = sched_get_priority_max(SCHED_RR);
	} else {
		sched.sched_priority = pri;
	}

	return sched_setscheduler(0, SCHED_RR, &sched);
}

/**
 * @brief Fail. Or not.
 * @param fatal
 * @param message
 * @param ...
 * @return
 */
int wiringPiFailure(int fatal, const char *message, ...) {
	va_list argp;
	char buffer[1024];

	if(!fatal && wiringPiReturnCodes) {
		return -1;
	}

	va_start (argp, message);
	vsnprintf(buffer, 1023, message, argp);
	va_end (argp);

	fprintf(stderr, "%s", buffer);
	exit(EXIT_FAILURE);

	return 0;
}

/*
 * setupCheck
 *	Another sanity check because some users forget to call the setup
 *	function. Mosty because they need feeding C drip by drip )-:
 *********************************************************************************
 */

static void setupCheck(const char *fName) {
	if(!wiringPiSetuped) {
		fprintf(stderr, "%s: You have not called one of the wiringPiSetup\n"
						"  functions, so I'm aborting your program before it crashes anyway.\n", fName);
		exit(EXIT_FAILURE);
	}
}

/*
 * gpioMemCheck:
 *	See if we're using the /dev/gpiomem interface, if-so then some operations
 *	can't be done and will crash the Pi.
 *********************************************************************************
 */

static void usingGpioMemCheck(const char *what) {
	if(usingGpioMem) {
		fprintf(stderr, "%s: Unable to do this when using /dev/gpiomem. Try sudo?\n", what);
		exit(EXIT_FAILURE);
	}
}

/*
 * piGpioLayout:
 *	Return a number representing the hardware revision of the board.
 *	This is not strictly the board revision but is used to check the
 *	layout of the GPIO connector - and there are 2 types that we are
 *	really interested in here. The very earliest Pi's and the
 *	ones that came after that which switched some pins ....
 *
 *	Revision 1 really means the early Model A and B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *		... and the Pi 2 - which is a B+ ++  ...
 *		... and the Pi 0 - which is an A+ ...
 *
 *	The main difference between the revision 1 and 2 system that I use here
 *	is the mapping of the GPIO pins. From revision 2, the Pi Foundation changed
 *	3 GPIO pins on the (original) 26-way header - BCM_GPIO 22 was dropped and
 *	replaced with 27, and 0 + 1 - I2C bus 0 was changed to 2 + 3; I2C bus 1.
 *
 *	Additionally, here we set the piModel2 flag too. This is again, nothing to
 *	do with the actual model, but the major version numbers - the GPIO base
 *	hardware address changed at model 2 and above (not the Zero though)
 *
 *********************************************************************************
 */

static void piGpioLayoutOops(const char *why) {
	fprintf(stderr, "Oops: Unable to determine board revision from /proc/cpuinfo\n");
	fprintf(stderr, " -> %s\n", why);
	fprintf(stderr, " ->  You'd best google the error to find out why.\n");
	//	fprintf(stderr, " ->  http://www.raspberrypi.org/phpBB3/viewtopic.php?p=184410#p184410\n");
	exit(EXIT_FAILURE);
}

int piGpioLayout() {
	FILE *cpuFd;
	char line[120];
	char *c;
	static int gpioLayout = -1;

	if(gpioLayout != -1) {    // No point checking twice
		return gpioLayout;
	}

	if((cpuFd = fopen("/proc/cpuinfo", "r")) == NULL) {
		piGpioLayoutOops("Unable to open /proc/cpuinfo");
	}

	// Start by looking for the Architecture to make sure we're really running
	//	on a Pi. I'm getting fed-up with people whinging at me because
	//	they can't get it to work on weirdFruitPi boards...

	while (fgets(line, 120, cpuFd) != NULL) {
		if(strncmp(line, "Hardware", 8) == 0) {
			break;
		}
	}

	if(strncmp(line, "Hardware", 8) != 0) {
		piGpioLayoutOops("No \"Hardware\" line");
	}

	if(wiringPiDebug) {
		printf("piGpioLayout: Hardware: %s\n", line);
	}

	// See if it's BCM2708 or BCM2709 or the new BCM2835.

	// OK. As of Kernel 4.8,  we have BCM2835 only, regardless of model.
	//	However I still want to check because it will trap the cheapskates and rip-
	//	off merchants who want to use wiringPi on non-Raspberry Pi platforms - which
	//	I do not support so don't email me your bleating whinges about anything
	//	other than a genuine Raspberry Pi.

#ifdef    DONT_CARE_ANYMORE
	if (! (strstr (line, "BCM2708") || strstr (line, "BCM2709") || strstr (line, "BCM2835")))
	{
	  fprintf (stderr, "Unable to determine hardware version. I see: %s,\n", line) ;
	  fprintf (stderr, " - expecting BCM2708, BCM2709 or BCM2835.\n") ;
	  fprintf (stderr, "If this is a genuine Raspberry Pi then please report this\n") ;
	  fprintf (stderr, "to projects@drogon.net. If this is not a Raspberry Pi then you\n") ;
	  fprintf (stderr, "are on your own as wiringPi is designed to support the\n") ;
	  fprintf (stderr, "Raspberry Pi ONLY.\n") ;
	  exit (EXIT_FAILURE) ;
	}
#endif

	// Actually... That has caused me more than 10,000 emails so-far. Mosty by
	//	people who think they know better by creating a statically linked
	//	version that will not run with a new 4.9 kernel. I utterly hate and
	//	despise those people.
	//
	//	I also get bleats from people running other than Raspbian with another
	//	distros compiled kernel rather than a foundation compiled kernel, so
	//	this might actually help them. It might not - I only have the capacity
	//	to support Raspbian.
	//
	//	However, I've decided to leave this check out and rely purely on the
	//	Revision: line for now. It will not work on a non-pi hardware or weird
	//	kernels that don't give you a suitable revision line.

	// So - we're Probably on a Raspberry Pi. Check the revision field for the real
	//	hardware type
	//	In-future, I ought to use the device tree as there are now Pi entries in
	//	/proc/device-tree/ ...
	//	but I'll leave that for the next revision. Or the next.

	// Isolate the Revision line

	rewind(cpuFd);
	while (fgets(line, 120, cpuFd) != NULL) {
		if(strncmp(line, "Revision", 8) == 0) {
			break;
		}
	}

	fclose(cpuFd);

	if(strncmp(line, "Revision", 8) != 0) {
		piGpioLayoutOops("No \"Revision\" line");
	}

	// Chomp trailing CR/NL

	for (c = &line[strlen(line) - 1]; (*c == '\n') || (*c == '\r'); --c) {
		*c = 0;
	}

	if(wiringPiDebug) {
		printf("piGpioLayout: Revision string: %s\n", line);
	}

	// Scan to the first character of the revision number

	for (c = line; *c; ++c) {
		if(*c == ':') {
			break;
		}
	}

	if(*c != ':') {
		piGpioLayoutOops("Bogus \"Revision\" line (no colon)");
	}

	// Chomp spaces

	++c;
	while (isspace (*c)) {
		++c;
	}

	if(!isxdigit (*c)) {
		piGpioLayoutOops("Bogus \"Revision\" line (no hex digit at start of revision)");
	}

	// Make sure its long enough

	if(strlen(c) < 4) {
		piGpioLayoutOops("Bogus revision line (too small)");
	}

	// Isolate  last 4 characters: (in-case of overvolting or new encoding scheme)

	c = c + strlen(c) - 4;

	if(wiringPiDebug) {
		printf("piGpioLayout: last4Chars are: \"%s\"\n", c);
	}

	if((strcmp(c, "0002") == 0) || (strcmp(c, "0003") == 0)) {
		gpioLayout = 1;
	} else {
		gpioLayout = 2;
	}    // Covers everything else from the B revision 2 to the B+, the Pi v2, v3, zero and CM's.

	if(wiringPiDebug) {
		printf("piGpioLayoutOops: Returning revision: %d\n", gpioLayout);
	}

	return gpioLayout;
}

/*
 * piBoardId:
 *	Return the real details of the board we have.
 *
 *	This is undocumented and really only intended for the GPIO command.
 *	Use at your own risk!
 *
 *	Seems there are some boards with 0000 in them (mistake in manufacture)
 *	So the distinction between boards that I can see is:
 *
 *		0000 - Error
 *		0001 - Not used
 *
 *	Original Pi boards:
 *		0002 - Model B,  Rev 1,   256MB, Egoman
 *		0003 - Model B,  Rev 1.1, 256MB, Egoman, Fuses/D14 removed.
 *
 *	Newer Pi's with remapped GPIO:
 *		0004 - Model B,  Rev 1.2, 256MB, Sony
 *		0005 - Model B,  Rev 1.2, 256MB, Egoman
 *		0006 - Model B,  Rev 1.2, 256MB, Egoman
 *
 *		0007 - Model A,  Rev 1.2, 256MB, Egoman
 *		0008 - Model A,  Rev 1.2, 256MB, Sony
 *		0009 - Model A,  Rev 1.2, 256MB, Egoman
 *
 *		000d - Model B,  Rev 1.2, 512MB, Egoman	(Red Pi, Blue Pi?)
 *		000e - Model B,  Rev 1.2, 512MB, Sony
 *		000f - Model B,  Rev 1.2, 512MB, Egoman
 *
 *		0010 - Model B+, Rev 1.2, 512MB, Sony
 *		0013 - Model B+  Rev 1.2, 512MB, Embest
 *		0016 - Model B+  Rev 1.2, 512MB, Sony
 *		0019 - Model B+  Rev 1.2, 512MB, Egoman
 *
 *		0011 - Pi CM,    Rev 1.1, 512MB, Sony
 *		0014 - Pi CM,    Rev 1.1, 512MB, Embest
 *		0017 - Pi CM,    Rev 1.1, 512MB, Sony
 *		001a - Pi CM,    Rev 1.1, 512MB, Egoman
 *
 *		0012 - Model A+  Rev 1.1, 256MB, Sony
 *		0015 - Model A+  Rev 1.1, 512MB, Embest
 *		0018 - Model A+  Rev 1.1, 256MB, Sony
 *		001b - Model A+  Rev 1.1, 256MB, Egoman
 *
 *	A small thorn is the olde style overvolting - that will add in
 *		1000000
 *
 *	The Pi compute module has an revision of 0011 or 0014 - since we only
 *	check the last digit, then it's 1, therefore it'll default to not 2 or
 *	3 for a	Rev 1, so will appear as a Rev 2. This is fine for the most part, but
 *	we'll properly detect the Compute Module later and adjust accordingly.
 *
 * And then things changed with the introduction of the v2...
 *
 * For Pi v2 and subsequent models - e.g. the Zero:
 *
 *   [USER:8] [NEW:1] [MEMSIZE:3] [MANUFACTURER:4] [PROCESSOR:4] [TYPE:8] [REV:4]
 *   NEW          23: will be 1 for the new scheme, 0 for the old scheme
 *   MEMSIZE      20: 0=256M 1=512M 2=1G
 *   MANUFACTURER 16: 0=SONY 1=EGOMAN 2=EMBEST
 *   PROCESSOR    12: 0=2835 1=2836
 *   TYPE         04: 0=MODELA 1=MODELB 2=MODELA+ 3=MODELB+ 4=Pi2 MODEL B 5=ALPHA 6=CM
 *   REV          00: 0=REV0 1=REV1 2=REV2
 *********************************************************************************
 */

void piBoardId(int *model, int *rev, int *mem, int *maker, int *warranty) {
	FILE *cpuFd;
	char line[120];
	char *c;
	unsigned int revision;
	int bRev, bType, bProc, bMfg, bMem, bWarranty;

	//	Will deal with the properly later on - for now, lets just get it going...
	//  unsigned int modelNum ;

	(void) piGpioLayout();    // Call this first to make sure all's OK. Don't care about the result.

	if((cpuFd = fopen("/proc/cpuinfo", "r")) == NULL) {
		piGpioLayoutOops("Unable to open /proc/cpuinfo");
	}

	while (fgets(line, 120, cpuFd) != NULL) {
		if(strncmp(line, "Revision", 8) == 0) {
			break;
		}
	}

	fclose(cpuFd);

	if(strncmp(line, "Revision", 8) != 0) {
		piGpioLayoutOops("No \"Revision\" line");
	}

	// Chomp trailing CR/NL

	for (c = &line[strlen(line) - 1]; (*c == '\n') || (*c == '\r'); --c) {
		*c = 0;
	}

	if(wiringPiDebug) {
		printf("piBoardId: Revision string: %s\n", line);
	}

	// Need to work out if it's using the new or old encoding scheme:

	// Scan to the first character of the revision number

	for (c = line; *c; ++c) {
		if(*c == ':') {
			break;
		}
	}

	if(*c != ':') {
		piGpioLayoutOops("Bogus \"Revision\" line (no colon)");
	}

	// Chomp spaces

	++c;
	while (isspace (*c)) {
		++c;
	}

	if(!isxdigit (*c)) {
		piGpioLayoutOops("Bogus \"Revision\" line (no hex digit at start of revision)");
	}

	revision = (unsigned int) strtol(c, NULL, 16); // Hex number with no leading 0x

	// Check for new way:
	if((revision & (1 << 23)) != 0) { // New way
		if(wiringPiDebug) {
			printf("piBoardId: New Way: revision is: %08X\n", revision);
		}

		bRev      = (revision & (0x0F << 0))  >> 0;
		bType     = (revision & (0xFF << 4))  >> 4;
		bProc     = (revision & (0x0F << 12)) >> 12; // Not used for now.
		bMfg      = (revision & (0x0F << 16)) >> 16;
		bMem      = (revision & (0x07 << 20)) >> 20;
		bWarranty = (revision & (0x03 << 24)) !=  0;

		*model    = bType;
		*rev      = bRev;
		*mem      = bMem;
		*maker    = bMfg;
		*warranty = bWarranty;

		if(wiringPiDebug) {
			printf("piBoardId: rev: %d, type: %d, proc: %d, mfg: %d, mem: %d, warranty: %d\n", bRev, bType, bProc, bMfg, bMem, bWarranty);
		}
	} else {  // Old way
		if(wiringPiDebug) {
			printf("piBoardId: Old Way: revision is: %s\n", c);
		}

		if(!isdigit (*c)) {
			piGpioLayoutOops("Bogus \"Revision\" line (no digit at start of revision)");
		}

		// Make sure its long enough
		if(strlen(c) < 4) {
			piGpioLayoutOops("Bogus \"Revision\" line (not long enough)");
		}

		// If longer than 4, we'll assume it's been over-volted
		*warranty = strlen(c) > 4;

		// Extract last 4 characters:
		c = c + strlen(c) - 4;

		// Fill out the reply's as appropriate
		if(strcmp(c, "0002") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0003") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_1;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0004") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "0005") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0006") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0007") == 0) {
			*model = PI_MODEL_A;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0008") == 0) {
			*model = PI_MODEL_A;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_SONY;;
		} else if(strcmp(c, "0009") == 0) {
			*model = PI_MODEL_A;
			*rev = PI_VERSION_1_2;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "000d") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "000e") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "000f") == 0) {
			*model = PI_MODEL_B;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0010") == 0) {
			*model = PI_MODEL_BP;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "0013") == 0) {
			*model = PI_MODEL_BP;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_EMBEST;
		} else if(strcmp(c, "0016") == 0) {
			*model = PI_MODEL_BP;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "0019") == 0) {
			*model = PI_MODEL_BP;
			*rev = PI_VERSION_1_2;
			*mem = 1;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0011") == 0) {
			*model = PI_MODEL_CM;
			*rev = PI_VERSION_1_1;
			*mem = 1;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "0014") == 0) {
			*model = PI_MODEL_CM;
			*rev = PI_VERSION_1_1;
			*mem = 1;
			*maker = PI_MAKER_EMBEST;
		} else if(strcmp(c, "0017") == 0) {
			*model = PI_MODEL_CM;
			*rev = PI_VERSION_1_1;
			*mem = 1;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "001a") == 0) {
			*model = PI_MODEL_CM;
			*rev = PI_VERSION_1_1;
			*mem = 1;
			*maker = PI_MAKER_EGOMAN;
		} else if(strcmp(c, "0012") == 0) {
			*model = PI_MODEL_AP;
			*rev = PI_VERSION_1_1;
			*mem = 0;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "0015") == 0) {
			*model = PI_MODEL_AP;
			*rev = PI_VERSION_1_1;
			*mem = 1;
			*maker = PI_MAKER_EMBEST;
		} else if(strcmp(c, "0018") == 0) {
			*model = PI_MODEL_AP;
			*rev = PI_VERSION_1_1;
			*mem = 0;
			*maker = PI_MAKER_SONY;
		} else if(strcmp(c, "001b") == 0) {
			*model = PI_MODEL_AP;
			*rev = PI_VERSION_1_1;
			*mem = 0;
			*maker = PI_MAKER_EGOMAN;
		} else {
			*model = 0;
			*rev = 0;
			*mem = 0;
			*maker = 0;
		}
	}
}

/**
 * @brief Select the native "balanced" mode, or standard mark:space mode
 * @param mode
 */
void pwmSetMode(int mode) {
	if((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO)) {
		if(mode == PWM_MODE_MS) {
			*(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE | PWM0_MS_MODE | PWM1_MS_MODE;
		} else {
			*(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE;
		}
	}
}

/**
 * @brief Set the PWM i register. We set both i registers to the same value.
 * @param i
 */
void pwmSetRange(unsigned int i) {
	if((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO)) {
		*(pwm + PWM0_RANGE) = i;
		delayMicroseconds(10);
		*(pwm + PWM1_RANGE) = i;
		delayMicroseconds(10);
	}
}

/**
 * @brief Set/Change the PWM clock.
 * @param divisor
 * @author Chris Hall <chris@kchall.plus.com>
 */
void pwmSetClock(int divisor) {
	uint32_t pwm_control;

	if(piGpioBase == GPIO_PERI_BASE_2711) {
		divisor = 540 * divisor / 192;
	}
	divisor &= 4095;

	if((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO)) {
		if(wiringPiDebug) {
			printf("Setting to: %d. Current: 0x%08X\n", divisor, *(clk + PWMCLK_DIV));
		}

		pwm_control = *(pwm + PWM_CONTROL);        // preserve PWM_CONTROL

		// We need to stop PWM prior to stopping PWM clock in MS mode otherwise BUSY
		// stays high.

		*(pwm + PWM_CONTROL) = 0;                // Stop PWM

		// Stop PWM clock before changing divisor. The delay after this does need to
		// this big (95uS occasionally fails, 100uS OK), it's almost as though the BUSY
		// flag is not working properly in balanced mode. Without the delay when DIV is
		// adjusted the clock sometimes switches to very slow, once slow further DIV
		// adjustments do nothing and it's difficult to get out of this mode.

		*(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x01;    // Stop PWM Clock
		delayMicroseconds(110);            // prevents clock going sloooow

		while ((*(clk + PWMCLK_CNTL) & 0x80) != 0) {    // Wait for clock to be !BUSY
			delayMicroseconds(1);
		}

		*(clk + PWMCLK_DIV) = BCM_PASSWORD | (divisor << 12);

		*(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x11;    // Start PWM clock
		*(pwm + PWM_CONTROL) = pwm_control;        // restore PWM_CONTROL

		if(wiringPiDebug) {
			printf("Set     to: %d. Now    : 0x%08X\n", divisor, *(clk + PWMCLK_DIV));
		}
	}
}

/**
 * @brief Set the frequency on a GPIO clock pin
 * @param pin
 * @param freq
 */
void gpioClockSet(int pin, int freq) {
	int divi, divr, divf;

	pin &= 63;

	/**/ if(wiringPiMode == WPI_MODE_PINS) {
		pin = pinToGpio[pin];
	} else if(wiringPiMode == WPI_MODE_PHYS) {
		pin = physToGpio[pin];
	} else if(wiringPiMode != WPI_MODE_GPIO) {
		return;
	}

	divi = 19200000 / freq;
	divr = 19200000 % freq;
	divf = (int) ((double) divr * 4096.0 / 19200000.0);

	if(divi > 4095) {
		divi = 4095;
	}

	*(clk + gpioToClkCon[pin]) = BCM_PASSWORD | GPIO_CLOCK_SOURCE;        // Stop GPIO Clock
	while ((*(clk + gpioToClkCon[pin]) & 0x80) != 0) {                // ... and wait
	}

	*(clk + gpioToClkDiv[pin]) = BCM_PASSWORD | (divi << 12) | divf;        // Set dividers
	*(clk + gpioToClkCon[pin]) = BCM_PASSWORD | 0x10 | GPIO_CLOCK_SOURCE;    // Start Clock
}

/**
 * @brief Locate our device node
 * @param pin
 * @return
 */
struct wiringPiNodeStruct *wiringPiFindNode(int pin) {
	struct wiringPiNodeStruct *node = wiringPiNodes;

	while (node != NULL) {
		if((pin >= node->pinBase) && (pin <= node->pinMax)) {
			return node;
		} else {
			node = node->next;
		}
	}

	return NULL;
}

/*!*********************************************************************************
 * Core Functions
 **********************************************************************************/

/**
 * @brief Sets the mode of a pin to be input, output or PWM output
 * @param pin
 * @param mode
 */
void pinMode(int pin, int mode) {
	int fSel, shift, alt;
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int origPin = pin;

	setupCheck("pinMode");

	if((pin & PI_GPIO_MASK) == 0)        // On-board pin
	{
		/**/ if(wiringPiMode == WPI_MODE_PINS) {
			pin = pinToGpio[pin];
		} else if(wiringPiMode == WPI_MODE_PHYS) {
			pin = physToGpio[pin];
		} else if(wiringPiMode != WPI_MODE_GPIO) {
			return;
		}

		softPwmStop(origPin);
		softToneStop(origPin);

		fSel = gpioToGPFSEL[pin];
		shift = gpioToShift[pin];

		/**/ if(mode == INPUT) {
			*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)); // Sets bits to zero = input
		} else if(mode == OUTPUT) {
			*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift);
		} else if(mode == SOFT_PWM_OUTPUT) {
			softPwmCreate(origPin, 0, 100);
		} else if(mode == SOFT_TONE_OUTPUT) {
			softToneCreate(origPin);
		} else if(mode == PWM_TONE_OUTPUT) {
			pinMode(origPin, PWM_OUTPUT);    // Call myself to enable PWM mode
			pwmSetMode(PWM_MODE_MS);
		} else if(mode == PWM_OUTPUT) {
			if((alt = gpioToPwmALT[pin]) == 0) {    // Not a hardware capable PWM pin
				return;
			}

			usingGpioMemCheck("pinMode PWM");

			// Set pin to PWM mode

			*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift);
			delayMicroseconds(110);        // See comments in pwmSetClockWPi

			pwmSetMode(PWM_MODE_BAL);    // Pi default mode
			pwmSetRange(1024);        // Default range of 1024
			pwmSetClock(32);        // 19.2 / 32 = 600KHz - Also starts the PWM
		} else if(mode == GPIO_CLOCK) {
			if((alt = gpioToGpClkALT0[pin]) == 0) {    // Not a GPIO_CLOCK pin
				return;
			}

			usingGpioMemCheck("pinMode CLOCK");

			// Set pin to GPIO_CLOCK mode and set the clock frequency to 100KHz

			*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift);
			delayMicroseconds(110);
			gpioClockSet(pin, 100000);
		}
	} else {
		if((node = wiringPiFindNode(pin)) != NULL) {
			node->pinMode(node, pin, mode);
		}
		return;
	}
}

/**
 * @brief Control the internal pull-up/down resistors on a GPIO pin.
 * @param pin
 * @param pud
 */
void pullUpDnControl(int pin, int pud) {
	struct wiringPiNodeStruct *node = wiringPiNodes;

	setupCheck("pullUpDnControl");

	if((pin & PI_GPIO_MASK) == 0)        // On-Board Pin
	{
		/**/ if(wiringPiMode == WPI_MODE_PINS) {
			pin = pinToGpio[pin];
		} else if(wiringPiMode == WPI_MODE_PHYS) {
			pin = physToGpio[pin];
		} else if(wiringPiMode != WPI_MODE_GPIO) {
			return;
		}

		if(piGpioPupOffset == GPPUPPDN0) {
			// Pi 4B pull up/down method
			int pullreg = GPPUPPDN0 + (pin >> 4);
			int pullshift = (pin & 0xf) << 1;
			unsigned int pullbits;
			unsigned int pull;

			switch (pud) {
				case PUD_OFF: pull = 0;
					break;
				case PUD_UP: pull = 1;
					break;
				case PUD_DOWN: pull = 2;
					break;
				default: return; /* An illegal value */
			}

			pullbits = *(gpio + pullreg);
			pullbits &= ~(3 << pullshift);
			pullbits |= (pull << pullshift);
			*(gpio + pullreg) = pullbits;
		} else {
			// legacy pull up/down method
			*(gpio + GPPUD) = pud & 3;
			delayMicroseconds(5);
			*(gpio + gpioToPUDCLK[pin]) = 1 << (pin & 31);
			delayMicroseconds(5);

			*(gpio + GPPUD) = 0;
			delayMicroseconds(5);
			*(gpio + gpioToPUDCLK[pin]) = 0;
			delayMicroseconds(5);
		}
	} else                        // Extension module
	{
		if((node = wiringPiFindNode(pin)) != NULL) {
			node->pullUpDnControl(node, pin, pud);
		}
		return;
	}
}

/**
 * @brief Read the value of a given Pin
 * @param pin
 * @return HIGH or LOW
 */
int digitalRead(int pin) {
	char c;
	struct wiringPiNodeStruct *node = wiringPiNodes;
	if((pin & PI_GPIO_MASK) == 0)        // On-Board Pin
	{
		/**/ if(wiringPiMode == WPI_MODE_GPIO_SYS)    // Sys mode
		{
			if(sysFds[pin] == -1) {
				return LOW;
			}

			lseek(sysFds[pin], 0L, SEEK_SET);
			read(sysFds[pin], &c, 1);
			return (c == '0') ? LOW : HIGH;
		} else if(wiringPiMode == WPI_MODE_PINS) {
			pin = pinToGpio[pin];
		} else if(wiringPiMode == WPI_MODE_PHYS) {
			pin = physToGpio[pin];
		} else if(wiringPiMode != WPI_MODE_GPIO) {
			return LOW;
		}

		if((*(gpio + gpioToGPLEV[pin]) & (1 << (pin & 31))) != 0) {
			return HIGH;
		} else {
			return LOW;
		}
	} else {
		if((node = wiringPiFindNode(pin)) == NULL) {
			return LOW;
		}
		return node->digitalRead(node, pin);
	}
}

/**
 * @brief Set an output bit
 * @param pin
 * @param value
 */
void digitalWrite(int pin, int value) {
	struct wiringPiNodeStruct *node = wiringPiNodes;

	if((pin & PI_GPIO_MASK) == 0) { // On-Board Pin
		/**/
		if(wiringPiMode == WPI_MODE_GPIO_SYS) { // Sys mode
			if(sysFds[pin] != -1) {
				if(value == LOW) {
					write(sysFds[pin], "0\n", 2);
				} else {
					write(sysFds[pin], "1\n", 2);
				}
			}
			return;
		} else if(wiringPiMode == WPI_MODE_PINS) {
			pin = pinToGpio[pin];
		} else if(wiringPiMode == WPI_MODE_PHYS) {
			pin = physToGpio[pin];
		} else if(wiringPiMode != WPI_MODE_GPIO) {
			return;
		}

		if(value == LOW) {
			*(gpio + gpioToGPCLR[pin]) = 1 << (pin & 31);
		} else {
			*(gpio + gpioToGPSET[pin]) = 1 << (pin & 31);
		}
	} else {
		if((node = wiringPiFindNode(pin)) != NULL) {
			node->digitalWrite(node, pin, value);
		}
	}
}

/**
 * @brief Initialise our start-of-time variable to be the current unix time in milliseconds and microseconds.
 */
static void initialiseEpoch(void) {
#ifdef    OLD_WAY
	struct timeval tv ;

	gettimeofday (&tv, NULL) ;
	epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
	epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
#else
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	epochMilli = (uint64_t) ts.tv_sec * (uint64_t) 1000 + (uint64_t) (ts.tv_nsec / 1000000L);
	epochMicro = (uint64_t) ts.tv_sec * (uint64_t) 1000000 + (uint64_t) (ts.tv_nsec / 1000L);
#endif
}

/**
 * @brief Wait for some number of milliseconds
 * @param howLong
 */
void delay(unsigned int howLong) {
	struct timespec sleeper, dummy;

	sleeper.tv_sec = (time_t) (howLong / 1000);
	sleeper.tv_nsec = (long) (howLong % 1000) * 1000000;

	nanosleep(&sleeper, &dummy);
}

/*!*********************************************************************************
 * delayMicroseconds:
 *	This is somewhat interesting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wasteful, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 **********************************************************************************/
void delayMicrosecondsHard(unsigned int howLong) {
	struct timeval tNow, tLong, tEnd;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec = howLong / 1000000;
	tLong.tv_usec = howLong % 1000000;
	timeradd (&tNow, &tLong, &tEnd);

	while (timercmp (&tNow, &tEnd, <)) {
		gettimeofday(&tNow, NULL);
	}
}

void delayMicroseconds(unsigned int howLong) {
	struct timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	/**/ if(howLong == 0) {
		return;
	} else if(howLong < 100) {
		delayMicrosecondsHard(howLong);
	} else {
		sleeper.tv_sec = wSecs;
		sleeper.tv_nsec = (long) (uSecs * 1000L);
		nanosleep(&sleeper, NULL);
	}
}

/**
 * @brief Milliseconds
 * @return Return a number of milliseconds as an unsigned int.
 * Wraps at 49 days.
 */
unsigned int millis() {
	uint64_t now;

#ifdef OLD_WAY
	struct timeval tv ;

	gettimeofday (&tv, NULL) ;
	now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;
#else
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	now = (uint64_t) ts.tv_sec * (uint64_t) 1000 + (uint64_t) (ts.tv_nsec / 1000000L);
#endif

	return (uint32_t) (now - epochMilli);
}

/**
 * @brief Must be called once at the start of your program execution.
 * @details
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 * memory mapped hardware directly.
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 * @return
 */
int wiringPiSetup() {
	int fd;
	int model, rev, mem, maker, overVolted;

	// It's actually a fatal error to call any of the wiringPiSetup routines more than once,
	//	(you run out of file handles!) but I'm fed-up with the useless twats who email
	//	me bleating that there is a bug in my code, so screw-em.

	if(wiringPiSetuped) {
		return 0;
	}

	wiringPiSetuped = TRUE;

	if(getenv(ENV_DEBUG) != NULL) {
		wiringPiDebug = TRUE;
	}

	if(getenv(ENV_CODES) != NULL) {
		wiringPiReturnCodes = TRUE;
	}

	if(wiringPiDebug) {
		printf("wiringPi: wiringPiSetup called\n");
	}

	// Get the board ID information. We're not really using the information here,
	//	but it will give us information like the GPIO layout scheme (2 variants
	//	on the older 26-pin Pi's) and the GPIO peripheral base address.
	//	and if we're running on a compute module, then wiringPi pin numbers
	//	don't really many anything, so force native BCM mode anyway.
	piBoardId(&model, &rev, &mem, &maker, &overVolted);

	if((model == PI_MODEL_CM) || (model == PI_MODEL_CM3) || (model == PI_MODEL_CM3P)) {
		wiringPiMode = WPI_MODE_GPIO;
	} else {
		wiringPiMode = WPI_MODE_PINS;
	}

	/**/
	if(piGpioLayout() == 1) { // A, B, Rev 1, 1.1
		pinToGpio = pinToGpioR1;
		physToGpio = physToGpioR1;
	} else { // A2, B2, A+, B+, CM, Pi2, Pi3, Zero
		pinToGpio = pinToGpioR2;
		physToGpio = physToGpioR2;
	}

	// ...

	switch (model) {
		case PI_MODEL_A:
		case PI_MODEL_B:
		case PI_MODEL_AP:
		case PI_MODEL_BP:
		case PI_ALPHA:
		case PI_MODEL_CM:
		case PI_MODEL_ZERO:
		case PI_MODEL_ZERO_W: {
			piGpioBase = GPIO_PERI_BASE_OLD;
			piGpioPupOffset = GPPUD;
		}
			break;
		case PI_MODEL_4B:
		case PI_MODEL_400:
		case PI_MODEL_CM4: {
			piGpioBase = GPIO_PERI_BASE_2711;
			piGpioPupOffset = GPPUPPDN0;
		}
			break;
		default: {
			piGpioBase = GPIO_PERI_BASE_2835;
			piGpioPupOffset = GPPUD;
		}
			break;
	}

	// Open the master /dev/ memory control device
	// Device strategy: December 2016:
	//	Try /dev/mem. If that fails, then
	//	try /dev/gpiomem. If that fails then game over.
	if((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
		if((fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC)) >= 0)    // We're using gpiomem
		{
			piGpioBase = 0;
			usingGpioMem = TRUE;
		} else {
			return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem or /dev/gpiomem: %s.\n"
											   "  Aborting your program because if it can not access the GPIO\n"
											   "  hardware then it most certianly won't work\n"
											   "  Try running with sudo?\n", strerror(errno));
		}
	}

	// Set the offsets into the memory interface.
	GPIO_PADS = piGpioBase + 0x00100000;
	GPIO_CLOCK_BASE = piGpioBase + 0x00101000;
	GPIO_BASE = piGpioBase + 0x00200000;
	GPIO_TIMER = piGpioBase + 0x0000B000;
	GPIO_PWM = piGpioBase + 0x0020C000;

	// Map the individual hardware components

	//	GPIO:
	gpio = (uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
	if(gpio == MAP_FAILED) {
		return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));
	}

	//	PWM
	pwm = (uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_PWM);
	if(pwm == MAP_FAILED) {
		return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror(errno));
	}

	//	Clock control (needed for PWM)
	clk = (uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_CLOCK_BASE);
	if(clk == MAP_FAILED) {
		return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror(errno));
	}

	//	The drive pads
	pads = (uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_PADS);
	if(pads == MAP_FAILED) {
		return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (PADS) failed: %s\n", strerror(errno));
	}

	//	The system timer
	timer = (uint32_t *) mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER);
	if(timer == MAP_FAILED) {
		return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (TIMER) failed: %s\n", strerror(errno));
	}

	// Set the timer to free-running, 1MHz.
	//	0xF9 is 249, the timer divide is base clock / (divide+1)
	//	so base clock is 250MHz / 250 = 1MHz.
	*(timer + TIMER_CONTROL) = 0x0000280;
	*(timer + TIMER_PRE_DIV) = 0x00000F9;
	timerIrqRaw = timer + TIMER_IRQ_RAW;

	initialiseEpoch();

	return 0;
}

/**
 * @brief Initialises the system into GPIO Pin mode and uses the memory mapped hardware directly.
 * @details Must be called once at the start of your program execution.
 * @return
 */
int wiringPiSetupGpio() {
	(void) wiringPiSetup();

	if(wiringPiDebug) {
		printf("wiringPi: wiringPiSetupGpio called\n");
	}

	wiringPiMode = WPI_MODE_GPIO;

	return 0;
}
