/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_reverb.c
 *
 * PROJECT....: F18.GR3
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * Apr 25, 2018  SoF    Module created.
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
void mod_reverb_effekt( fp_sample_t *in, fp_sample_t *out)
{
  if(is_sw1_pressed())
  {
  const float in_gain = -0.45;
  const float fb_gain = -0.45;
  const uint16_t delay = 1500;

  fp_sample_t fp_sample;
  sample_buffer_get_z( &fp_sample, delay);

  out->left_fp32 = in->left_fp32 + fp_sample.left_fp32;
  out->right_fp32 = in->right_fp32 + fp_sample.right_fp32;

  fp_sample.left_fp32 = ((out->left_fp32 * fb_gain) + (in->left_fp32 * in_gain));
  fp_sample.right_fp32 = ((out->right_fp32 * fb_gain) + (in->right_fp32 * in_gain));
  sample_buffer_put(&fp_sample);

  }
  else
  {
    out->left_fp32 = in->left_fp32;
    out->right_fp32 = in->right_fp32;
  }
}

void mod_reverb_init( void )
{

}

void mod_reverb_ui( void )
{

}


/****************************** End Of Module *******************************/
