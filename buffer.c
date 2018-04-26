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
#define   BUFFER_SIZE   2000

/*****************************   Constants   ********************************/

/*****************************   Variables   ********************************/
fp_sample_t buffer[BUFFER_SIZE];
uint16_t head = 0; //pointer to head in buffer
uint8_t flip = 0;
/*****************************   Functions   *********************************
 *   Function : See General module specification (general.h-file).
 *****************************************************************************/

void sample_buffer_put(fp_sample_t *data)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer.
 *******************************************************************************/
{
  buffer[head].left_fp32 = data->left_fp32;
  buffer[head].right_fp32 = data->right_fp32;

  if (head < (BUFFER_SIZE-1))
    head++;
  else
  {
    flip = 1;
    head = 0;
  }
}

void sample_buffer_get(fp_sample_t *data)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer.
 *******************************************************************************/
{
  data->left_fp32 = buffer[head].left_fp32;
  data->right_fp32 = buffer[head].right_fp32;
}

void sample_buffer_put_z(fp_sample_t *data, uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data should be put into and the data to put in.
 *    Output   :
 *    Function : Inserts data into the buffer at location offset from header by z.
 *******************************************************************************/
{
  uint16_t index;

  if( z < BUFFER_SIZE )
  {
    index = head > z ? head - z : (BUFFER_SIZE - z) + head;
    buffer[index].left_fp32 = data->left_fp32;
    buffer[index].right_fp32 = data->right_fp32;
  }
}

void sample_buffer_get_z(fp_sample_t *data, uint16_t z)
/*****************************************************************************
 *    Input    : The buffer data is to be got from.
 *    Output   :
 *    Function : Gets data from buffer at location offset from header by z.
 *******************************************************************************/
{
  uint16_t index;

  if ((head > z) && !flip)
  {
    data->left_fp32 = 0;
    data->right_fp32 = 0;
  }
  else
  {
    index = head > z ? head - z : (BUFFER_SIZE - z) + head;
    data->left_fp32 = buffer[index].left_fp32;
    data->right_fp32 = buffer[index].right_fp32;
  }
}

/****************************** End Of Module *******************************/
