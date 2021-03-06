/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_echo.h
 *
 * PROJECT....: 
 *
 * DESCRIPTION:
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * Apr 24, 2018	jorn    Module created.
 * Apr 27, 2018 SoF     Functions redesigned.
 *
 *****************************************************************************/

#ifndef MOD_ECHO_H_
#define MOD_ECHO_H_

/***************************** Include files *******************************/
#include <stdint.h>
#include "buffer.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void mod_echo_effekt( fp_sample_t *in, fp_sample_t *out);
/*****************************************************************************
 *   Input    : Pointer to input and return sample of type sample_t.
 *   Output   : -
 *   Function : Create echo effect from supplied sample.
 ******************************************************************************/

void mod_echo_init( void );
/*****************************************************************************
 *   Input    :
 *   Output   :
 *   Function : Initialize the echo module.
 ******************************************************************************/

BOOLEAN mod_echo_setdelay();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Handles changing the delay value used by "mod_echo_effekt".
 *******************************************************************************/

BOOLEAN mod_echo_setgain();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Handles changing the gain value used by "mod_echo_effekt".
 *******************************************************************************/


/****************************** End Of Module *******************************/
#endif /* MOD_ECHO_H_ */
