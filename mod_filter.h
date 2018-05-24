/*****************************************************************************
* University of Southern Denmark
* Embedded Programming (EMP)
*
* MODULENAME.: mod_filter.h
*
* PROJECT....: F18.GR3.Code
*
* DESCRIPTION: Describe something.
*
* DEPENDENCIES: General.
*
* Change Log:
******************************************************************************
* Date    		Id    Change
* MMM DD, YYYY
* ----------------------------------------------------------------------------
* 20 May 2018	FrH   Module created.
*
*****************************************************************************/

#ifndef _MOD_FILTER_H_
  #define _MOD_FILTER_H_

/***************************** Include files ********************************/
#include <stdint.h>
#include "buffer.h"
#include "hid.h"
#include "lcd1602.h"
#include "audio.h"

/*****************************    Defines    ********************************/
/********************** External declaration of Variables *******************/

/*****************************   Constants   ********************************/

/*************************  Function interfaces *****************************/

void mod_filter(fp_sample_t *in, fp_sample_t *out);
/*****************************************************************************
*   Input    : Pointer to input and return sample of type sample_t.
*   Output   : -
*   Function : Performs a filtering on the input.
*****************************************************************************/


BOOLEAN mod_filter_setfcutoff(void);
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Sets the cutoff frequency
*****************************************************************************/

void mod_filter_init(void);
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Initializes the filter and calculates the coefficients.
*****************************************************************************/

/****************************** End Of Module *******************************/
#endif

