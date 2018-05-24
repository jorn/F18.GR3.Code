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
 * Apr 20, 2018	SoF    Module created.
 *
 *****************************************************************************/

#ifndef REBUF_H_
#define REBUF_H_

/***************************** Include files *******************************/
#include <stdint.h>
#include "buffer.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void re_buffer_get(fp_sample_t * data);
/*****************************************************************************
 *    Input    : The buffer data to be got from buffer.
 *    Output   : 
 *    Function : Gets data from buffer.
 *******************************************************************************/

void re_buffer_get_out(fp_sample_t *data);
/*****************************************************************************
 *    Input    : The buffer data to be got from buffer.
 *    Output   :
 *    Function : Gets data from buffer, clears drawer, then increments pointer.
 *******************************************************************************/

void re_buffer_put_z(fp_sample_t *data, uint16_t z);
/*****************************************************************************
 *    Input    : Data to be put in buffer, and number of drawers ahead of pointer.
 *    Output   :
 *    Function : Inserts data into the buffer Z drawers ahead of the pointer.
 *******************************************************************************/

#endif /* REBUF_H_ */
