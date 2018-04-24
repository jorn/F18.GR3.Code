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
#include "buffer.h"
#include "hardware.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/
void mod_echo_effekt( fp_sample_t *in, fp_sample_t *out)
{
  if(is_digi_p2_pressed())
  {
  const float gain = .5;
  const uint16_t delay = 1999;

  fp_sample_t fp_sample;
  sample_buffer_get_z( &fp_sample, delay);

  out->left_fp32 = in->left_fp32 + (fp_sample.left_fp32 * gain);
  out->right_fp32 = in->right_fp32 + (fp_sample.right_fp32 * gain);
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
