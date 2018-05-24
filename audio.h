/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: audio.h
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION:
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * Apr 19, 2018	jorn    Module created.
 *
 *****************************************************************************/

#ifndef AUDIO_H_
#define AUDIO_H_

/***************************** Include files *******************************/
#include "emp_type.h"

/*****************************    Defines    *******************************/


/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/
void audio_init();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Init the audio module and configures the default setting
 *******************************************************************************/

uint8_t get_cpu_load();
/*****************************************************************************
 *    Input    : -
 *    Output   : % Audio CPU load
 *    Function : Get %Audio CPU load - based on ticks used by sample handler
 *******************************************************************************/

void audio_pwm_on();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Turns PWM audio out on
 *******************************************************************************/

void audio_pwm_off();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Turns PWM audio out off
 *******************************************************************************/

float get_sample(uint16_t offset);
/*****************************************************************************
 *    Input    : The sample offset
 *    Output   : The sample at OFFSET from HEAD sample
 *    Function : Returns the sample OFFSET from HEAD sample
 *******************************************************************************/

/****************************** End Of Module *******************************/
#endif /* AUDIO_H_ */
