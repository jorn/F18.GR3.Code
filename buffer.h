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
#include <stdint.h>

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/
typedef struct
{
    float left_fp32;
    float right_fp32;
} fp_sample_t;

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void sample_buffer_put(fp_sample_t *data);
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   : 
 *    Function : Inserts data into the buffer.
 *******************************************************************************/

void sample_buffer_get(fp_sample_t * data);
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   : 
 *    Function : Gets data from buffer.
 *******************************************************************************/

void sample_buffer_put_z(fp_sample_t *data, uint16_t z);
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/

void sample_buffer_get_z(fp_sample_t *data, uint16_t z);
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer.
 *******************************************************************************/

#endif /* BUFFER_H_ */
