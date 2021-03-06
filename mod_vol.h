/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_vol.h
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
 *
 *****************************************************************************/

#ifndef MOD_VOL_H_
#define MOD_VOL_H_

/***************************** Include files *******************************/
#include <stdint.h>
#include "buffer.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void mod_vol_effekt( fp_sample_t *in, fp_sample_t *out);
/*****************************************************************************
 *   Input    : Pointer to input and return sample of type sample_t.
 *   Output   : -
 *   Function : Modify amplitude of sample.
 ******************************************************************************/

void mod_vol_init( void );
/*****************************************************************************
 *   Input    :
 *   Output   :
 *   Function : Initialize the volume control module.
 ******************************************************************************/

BOOLEAN mod_vol_setvol();
/*****************************************************************************
 *   Input    : 
 *   Output   : 
 *   Function : Handles changing the value used by "mod_vol_effekt" to modify amplitude.
 ******************************************************************************/


/****************************** End Of Module *******************************/
#endif /* MOD_VOL_H_ */
