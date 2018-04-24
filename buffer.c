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
fp_sample_t buffer[4000];
uint16_t head = 0; //pointer to head in buffer
uint16_t size = 4000;
uint8_t flip = 0;
/*****************************   Functions   *********************************
 *   Function : See General module specification (general.h-file).
 *****************************************************************************/

int8_t sample_buffer_put(fp_sample_t *data)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/
{
    if (head < size)
    {
        buffer[head].left_fp32 = data->left_fp32;
        buffer[head].right_fp32 = data->right_fp32;
        head++;
    }
    else
    {
        if (!(flip))
        {
            flip = 1;
        }
        head = 0;
    }
}

int8_t sample_buffer_get(fp_sample_t * data)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer.
 *******************************************************************************/
{
    *data->left_fp32 = buffer[(head - 1)].left_fp32;
    *data->right_fp32 = buffer[(head - 1)].right_fp32;
    return 0;
}

int8_t sample_buffer_put_z(fp_sample_t *data, uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer at location offset from header by z.
 *******************************************************************************/
{
    if (head > z)
        uint16_t addr = (head - z);
    else
    {
        uint16_t addr = ((size - z) + head);
    }
    buffer[addr].left_fp32 = *data->left_fp32;
    buffer[addr].right_fp32 = data->right_fp32;
}

int8_t sample_buffer_get_z(fp_sample_t *data, uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer at location offset from header by z.
 *******************************************************************************/
{
    if (z > head)
    {
        if (flip)
        {
            uint16_t addr = ((size - z) + head);
            *data->left_fp32 = buffer[addr].left_fp32;
            *data->right_fp32 = buffer[addr].right_fp32;
        }
        else
        {
            *data->left_fp32 = 0;
            *data->right_fp32 = 0;
        }
    }
    else
    {
        *data->left_fp32 = buffer[(head - z)].left_fp32;
        *data->right_fp32 = buffer[(head - z)].right_fp32;
    }
}

/****************************** End Of Module *******************************/
