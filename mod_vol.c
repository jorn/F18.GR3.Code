/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: mod_vol.c
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

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/
void mod_vol_effekt( fp_sample_t *in, fp_sample_t *out)
{
  const float gain = 1;

  out->left_fp32 = in->left_fp32 * gain;
  out->right_fp32 = in->right_fp32 * gain;
}

void mod_vol_init( void )
{

}

void mod_vol_ui( void )
{

}


/****************************** End Of Module *******************************/
