/*****************************************************************************
 * University of Southern Denmark
 * Embedded Programming (EMP)
 *
 * MODULENAME.: buffer.c
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION: See module specification file (.h-file).
 *
 * Change Log:
 ******************************************************************************
 * Date			Id	  Change
 * MMM DD, YYYY
 * ----------------------------------------------------------------------------
 * Apr 24, 2018	SoF   Module created.
 *
 *****************************************************************************/

/***************************** Include files ********************************/
#include "buffer.h"
/*****************************    Defines    ********************************/
typedef struct
{
    float left_fp32;
    float right_fp32;
} fp_sample_t;

/*****************************   Constants   ********************************/

/*****************************   Variables   ********************************/

/*****************************   Functions   *********************************
 *   Function : See General module specification (general.h-file).
 *****************************************************************************/

int8_t sample_buffer_reset(sample_buffer_t * sample_buffer)
/*****************************************************************************
 *    Input    : The buffer to be reset
 *    Output   :
 *    Function : Resets the buffer.
 *******************************************************************************/
{

}

int8_t sample_buffer_put(sample_buffer_t * sample_buffer, fp_sample_t data)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/
{
    if (sample_buffer->head < sample_buffer->size)
    {
        sample_buffer->buffer[sample_buffer->head] = data;
        sample_buffer->head++;
    }
    else
    {
        if (!(sample_buffer->flip))
        {
            sample_buffer->flip = 1;
        }
        sample_buffer->head = 0;
    }
}

int8_t sample_buffer_get(sample_buffer_t * sample_buffer, fp_sample_t * data)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer.
 *******************************************************************************/
{

}

int8_t sample_buffer_put_z(sample_buffer_t * sample_buffer, fp_sample_t data,
                           uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer at location offset from header by z.
 *******************************************************************************/
{
    if (sample_buffer->head > z)
        uint16_t addr = (sample_buffer->head - z);
    else
    {
        uint16_t addr = ((sample_buffer->size - z) + sample_buffer->head);
    }
    sample_buffer->buffer[addr] = data;
}

int8_t sample_buffer_get_z(sample_buffer_t * sample_buffer, fp_sample_t * data,
                           uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer at location offset from header by z.
 *******************************************************************************/
{

}

bool sample_buffer_empty(sample_buffer_t sample_buffer)
/*****************************************************************************
 *    Input    : The buffer to check.
 *    Output   : Outputs 1 if buffer is empty and 0 if not.
 *    Function : Checks if a buffer is empty.
 *******************************************************************************/
{

}

bool sample_buffer_full(sample_buffer_t sample_buffer)
/*****************************************************************************
 *    Input    : The buffer to check.
 *    Output   : Outputs 1 if buffer is full and 0 if not.
 *    Function : Checks if a buffer is full.
 *******************************************************************************/
{

}

/****************************** End Of Module *******************************/
