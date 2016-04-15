/*
 * ws_func.c
 *
 *  Created on: Mar 10, 2016
 *      Author: CNWELI4
 */

#include "flowsensor.h"
#include "hw_gpio.h"


flowrate_typedef fls_1;
flowrate_typedef fls_2;
GPIO_pinInfo pin_fs_input_1;
GPIO_pinInfo pin_fs_input_2;
static uint32_t pre_cap_time_1 = 0;
static uint32_t pre_cap_time_2 = 0;

// init the reading of flow sensor
void flowsensor_init ()
{
	// Temp variable for storing PinInfo
	GPIO_pinInfo tempPinInfo;

	GPIO_TagConfigProperties(GPIOTag_FLOWSENSOR_1, GPIO_SET_IN_FLOATING, GPIO_SPD_HIGH);
	tempPinInfo = GPIO_ConfigInterrupt(GPIOTag_FLOWSENSOR_1, GPIO_TRG_RISING, FLOWSENSOR_IRQ);
	pin_fs_input_1 = tempPinInfo;

	GPIO_TagConfigProperties(GPIOTag_FLOWSENSOR_2, GPIO_SET_IN_PULLUP, GPIO_SPD_HIGH);
	tempPinInfo = GPIO_ConfigInterrupt(GPIOTag_FLOWSENSOR_2, GPIO_TRG_RISING, FLOWSENSOR_IRQ);
	pin_fs_input_2 = tempPinInfo;

	// init the 2 sensor's alive-monitor
	alive_monitor_1.AliveFlag = 0;
	alive_monitor_1.IsAwaken = 0;
	alive_monitor_1.TrigCount = 0;
	alive_monitor_1.TrigThreshold = 50;			// the lowest working freq of sensor is 20Hz

	alive_monitor_2.AliveFlag = 0;
	alive_monitor_2.IsAwaken = 0;
	alive_monitor_2.TrigCount = 0;
	alive_monitor_2.TrigThreshold = 50;			// the lowest working freq of sensor is 20Hz
}

// ISR for GPIO PD
void flowsensor_reading ()
{
	uint32_t cap_time_1 = 0;
	uint32_t cap_time_2 = 0;

	// Verify that one of our pin generated the interrupt
	if ((HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_MIS) & 0x00000001)) {
		// this means flow sensor 1 has captured an pulse
		// clear the int status bit firstly.
		GPIOIntClear(GPIO_PORTD_AHB_BASE, 0x00000001);
		alive_monitor_1.IsAwaken = 1;

		cap_time_1 = Time_GetMs();
		// if just resumed from last "inactive", set freq to 0.
		if (alive_monitor_1.AliveFlag == 1) {
			fls_1.pulse_freq = 1000/(cap_time_1 - pre_cap_time_1);
		}
		else {
			fls_1.pulse_freq = 0;
		}
		pre_cap_time_1 = cap_time_1;
	}
	else if (HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_MIS) & 0x00000002) {
		// this means flow sensor 2 has captured an pulse
		// clear the int status bit firstly.
		GPIOIntClear(GPIO_PORTD_AHB_BASE, 0x00000002);
		alive_monitor_2.IsAwaken = 1;

		// this means flow sensor 1 has captured an pulse
		cap_time_2 = Time_GetMs();
		// if just resumed from last "inactive", set freq to 0.
		if (alive_monitor_2.AliveFlag == 1) {
			fls_2.pulse_freq = 1000/(cap_time_2 - pre_cap_time_2);
		}
		else {
			fls_2.pulse_freq = 0;
		}
		pre_cap_time_2 = cap_time_2;
	}
	else return;
}
