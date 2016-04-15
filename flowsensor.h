/*
 * flowsensor.h
 *
 *  Created on: Mar 21, 2016
 *      Author: CNWELI4
 */

#ifndef FLOWSENSOR_H_
#define FLOWSENSOR_H_

#include "bsp_GPIO.h"

typedef struct
{
  uint32_t pulse_count;
  uint16_t pulse_freq;
  double volumerate_lpm;
} flowrate_typedef;

extern flowrate_typedef fls_1;
extern flowrate_typedef fls_2;
extern GPIO_pinInfo pin_fs_input_1;
extern GPIO_pinInfo pin_fs_input_2;

extern void flowsensor_init (void);

#endif /* FLOWSENSOR_H_ */
