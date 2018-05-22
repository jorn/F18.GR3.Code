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
#include "rebuf.h"
#include "buffer.h"

/*****************************    Defines    ********************************/
#define   RE_BUFFER_SIZE   2001

/*****************************   Constants   ********************************/

/*****************************   Variables   ********************************/
fp_sample_t effect_buffer[RE_BUFFER_SIZE];
uint16_t now = 0; //pointer to head in buffer
/*****************************   Functions   *********************************
 *   Function : See General module specification (general.h-file).
 *****************************************************************************/

void re_buffer_get(fp_sample_t *data)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/
{
    data->left_fp32 = effect_buffer[now].left_fp32;
    data->right_fp32 = effect_buffer[now].right_fp32;
}

void re_buffer_get_out(fp_sample_t *data)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/
{
    data->left_fp32 = effect_buffer[now].left_fp32;
    data->right_fp32 = effect_buffer[now].right_fp32;

    effect_buffer[now].left_fp32 = 0;
    effect_buffer[now].right_fp32 = 0;

    if (now < (RE_BUFFER_SIZE - 1))
        now++;
    else
    {
        now = 0;
    }
}


void re_buffer_put_z(fp_sample_t *data, uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer at location offset from header by z.
 *******************************************************************************/
{
    uint16_t index;

    if (z < RE_BUFFER_SIZE)
    {
        index = (now + z) <= (RE_BUFFER_SIZE - 1) ? now + z : (now + z) - RE_BUFFER_SIZE;
        if (effect_buffer[index].left_fp32 || effect_buffer[index].right_fp32)
        {
            effect_buffer[index].left_fp32 = ((effect_buffer[index].left_fp32 / 2) + (data->left_fp32 / 2));
            effect_buffer[index].right_fp32 = ((effect_buffer[index].right_fp32 / 2) + (data->right_fp32 / 2));
        }
        else
        {
            effect_buffer[index].left_fp32 += data->left_fp32;
            effect_buffer[index].right_fp32 += data->right_fp32;
        }
    }
}

/****************************** End Of Module *******************************/
