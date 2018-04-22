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
void audio_pwm_on();
void audio_pwm_off();

/****************************** End Of Module *******************************/
#endif /* AUDIO_H_ */
