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
 *    Input    : The buffer data is to be got from.
 *    Output   : 
 *    Function : Gets data from buffer.
 *******************************************************************************/

void re_buffer_get_out(fp_sample_t *data);
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer, zeroes the drawer.
 *******************************************************************************/

void re_buffer_put_z(fp_sample_t *data, uint16_t z);
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/

#endif /* REBUF_H_ */
