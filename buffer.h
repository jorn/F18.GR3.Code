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

#ifndef BUFFER_H_
#define BUFFER_H_

/***************************** Include files *******************************/
#include "emp_type.h"
#include <stdint.h>
/*****************************    Defines    *******************************/


/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

int8_t sample_buffer_reset(sample_buffer_t * sample_buffer);
/*****************************************************************************
 *    Input    : The buffer to be reset
 *    Output   : 
 *    Function : Resets the buffer.
 *******************************************************************************/

int8_t sample_buffer_put(sample_buffer_t * sample_buffer, float data);
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   : 
 *    Function : Inserts data into the buffer.
 *******************************************************************************/

int8_t sample_buffer_get(sample_buffer_t * sample_buffer, float * data);
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   : 
 *    Function : Gets data from buffer.
 *******************************************************************************/

int8_t sample_buffer_put_z(sample_buffer_t * sample_buffer, float data);
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/

int8_t sample_buffer_get_z(sample_buffer_t * sample_buffer, float * data);
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer.
 *******************************************************************************/

bool sample_buffer_empty(sample_buffer_t sample_buffer);
/*****************************************************************************
 *    Input    : The buffer to check.
 *    Output   : Outputs 1 if buffer is empty and 0 if not.
 *    Function : Checks if a buffer is empty.
 *******************************************************************************/

bool sample_buffer_full(sample_buffer_t sample_buffer);
/*****************************************************************************
 *    Input    : The buffer to check.
 *    Output   : Outputs 1 if buffer is full and 0 if not.
 *    Function : Checks if a buffer is full.
 *******************************************************************************/

/****************************** End Of Module *******************************/
#endif /* BUFFER_H_ */
