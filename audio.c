/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: audio.c
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * Apr 19, 2018  jorn    Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "audio.h"
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "hardware.h"
#include "global.h"
#include "mod_vol.h"
#include "mod_echo.h"
#include "mod_reverb.h"
#include "mod_filter.h"
#include "buffer.h"
#include "rebuf.h"

/*****************************    Defines    *******************************/
#define     MCB_POOL_SIZE     10
/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
typedef enum{
  ON,
  OFF
}state_t;

typedef struct {
  BOOLEAN     stereo;
  BOOLEAN     dac;
  BOOLEAN     pwm;
} acb_t;        // Audio Control Block type

static acb_t acb;

typedef struct {
  BOOLEAN     active;
  void        (*module)(fp_sample_t*, fp_sample_t*);
} mcb_t;

static mcb_t mcb_pool[MCB_POOL_SIZE];

uint8_t cpu_load = 0;

/*****************************   Functions   *******************************/
uint8_t get_cpu_load()
{
  return cpu_load;
}

void sample_handler( void )
{
  sample_int_clear();

  static sample_t sample;
  static fp_sample_t fp_sample_in;
  static fp_sample_t fp_sample_out;

  sample_in( &sample );
  if(!acb.stereo)
  {
    sample.right = sample.left;
  }

  fp_sample_in.left_fp32 = (float)((sample.left) - 2048);
  fp_sample_in.right_fp32 = (float)((sample.right) - 2048);

  sample_buffer_put( &fp_sample_in );

  for(int8_t i=0; i<MCB_POOL_SIZE; i++)
  {
    if(mcb_pool[i].active)
    {
      mcb_pool[i].module( &fp_sample_in, &fp_sample_out );
      // flip in and out
      fp_sample_in = fp_sample_out;
    }
  }

  re_buffer_get_out( &fp_sample_in );
  fp_sample_out.left_fp32 = fp_sample_in.left_fp32 + fp_sample_out.left_fp32;
  fp_sample_out.right_fp32 = fp_sample_in.right_fp32 + fp_sample_out.right_fp32;

  // add 11 bit offset
  fp_sample_out.left_fp32 += 2048;
  fp_sample_out.right_fp32 += 2048;

  // truncate and cast sample
  sample.left = fp_sample_out.left_fp32 > 4096 ? 4096 : (INT16U)fp_sample_out.left_fp32 ;
  sample.right = fp_sample_out.right_fp32 > 4096 ? 4096 : (INT16U)fp_sample_out.right_fp32 ;

  if( acb.pwm )
    sample_out_pwm( &sample );

  if( acb.dac )
    sample_out_spi( &sample );

  cpu_load = (TIMER3_TAR_R * 100) / TIMER3_TAILR_R;
}

void audio_pwm_on()
{
  acb.pwm = TRUE;
}

void audio_pwm_off()
{
  acb.pwm = FALSE;
}

void audio_init()
{
  acb.stereo = TRUE;
  acb.dac = TRUE;
  acb.pwm = TRUE;

  // Set PB3 low for LDAC
  bit_clear( GPIO_PORTB_DATA_R, BIT_3);

  mcb_pool[0].active = FALSE;
  mcb_pool[0].module = mod_reverb_effekt;
  mcb_pool[1].active = TRUE;
  mcb_pool[1].module = mod_echo_effekt;

  // Add volume module at the end
  mcb_pool[MCB_POOL_SIZE-1].active = TRUE;
  mcb_pool[MCB_POOL_SIZE-1].module = mod_vol_effekt;

  mod_vol_init();
  mod_echo_init();
  }






/****************************** End Of Module *******************************/
