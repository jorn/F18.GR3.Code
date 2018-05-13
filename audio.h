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
uint8_t get_cpu_load();
void audio_pwm_on();
void audio_pwm_off();
float get_sample(uint16_t offset);
/*****************************************************************************
 *    Input    : The sample offset
 *    Output   : The sample at OFFSET from HEAD sample
 *    Function : Returns the sample OFFSET from HEAD sample
 *******************************************************************************/
void echo_effect(float decay, uint16_t delay);
void reverb_effect(float decay1, float decay2, uint16_t delay);

/****************************** End Of Module *******************************/
#endif /* AUDIO_H_ */
