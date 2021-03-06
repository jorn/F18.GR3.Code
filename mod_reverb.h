/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_reverb.h
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
 * Apr 25, 2018	SoF    Module created.
 * Apr 27, 2018 SoF    Functions redesigned.
 *
 *****************************************************************************/

#ifndef MOD_REVERB_H_
#define MOD_REVERB_H_

/***************************** Include files *******************************/
#include <stdint.h>
#include "buffer.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void mod_reverb_effekt( fp_sample_t *in, fp_sample_t *out);
/*****************************************************************************
 *   Input    : Pointer to input and return sample of type sample_t.
 *   Output   : -
 *   Function : Create reverb effect from supplied sample.
 ******************************************************************************/

void mod_reverb_init( void );
/*****************************************************************************
 *   Input    :
 *   Output   :
 *   Function : Initialize the reverb module.
 ******************************************************************************/

BOOLEAN mod_reverb_setgain();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Handles changing the gain value used by "mod_reverb_effekt".
 *******************************************************************************/

BOOLEAN mod_reverb_setdelay();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Handles changing the delay value used by "mod_reverb_effekt".
 *******************************************************************************/

/****************************** End Of Module *******************************/
#endif /* MOD_REVERB_H_ */
