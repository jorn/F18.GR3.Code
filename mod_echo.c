/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_echo.c
 *
 * PROJECT....: F18.GR3
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * Apr 24, 2018  jorn    Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "audio.h"
#include "rebuf.h"
#include "hardware.h"
#include "buffer.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/
void mod_echo_effekt( fp_sample_t *in, fp_sample_t *out)
{
  if(is_sw2_pressed())
  {
  const float gain = .5;
  const uint16_t delay = 2000;

  fp_sample_t fp_sample;

  fp_sample.left_fp32 = (in->left_fp32 * gain);
  fp_sample.right_fp32 = (in->right_fp32 * gain);

  re_buffer_put_z(&fp_sample, delay);

  out->left_fp32 = in->left_fp32;
  out->right_fp32 = in->right_fp32;
  }
  else
  {
    out->left_fp32 = in->left_fp32;
    out->right_fp32 = in->right_fp32;
  }
}

void mod_echo_init( void )
{

}

void mod_echo_ui( void )
{

}


/****************************** End Of Module *******************************/
