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
#include "rebuf.h"
#include "hardware.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/
void mod_reverb_effekt( fp_sample_t *in, fp_sample_t *out)
{
  if(is_digi_p2_pressed())
  {
  const float in_gain = -0.45;
  const float fb_gain = -0.50;
  const uint16_t delay = 2000;

  fp_sample_t fp_sample;
  re_buffer_get(&fp_sample);

  fp_sample.left_fp32 = ((in->left_fp32 * in_gain) + ((in->left_fp32 + fp_sample.left_fp32) * fb_gain));
  fp_sample.right_fp32 = ((in->right_fp32 * in_gain) + ((in->left_fp32 + fp_sample.right_fp32) * fb_gain));

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

void mod_reverb_init( void )
{

}

void mod_reverb_ui( void )
{

}


/****************************** End Of Module *******************************/
