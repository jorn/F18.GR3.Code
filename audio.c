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

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
static volatile sample_t *sample;

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



/*****************************   Functions   *******************************/
void sample_handler( void )
{
  sample_int_clear();

  debug_pins_high( DEBUG_P1 );

  sample_in( &sample );

  if( acb.pwm )
    sample_out_pwm( &sample );

  if( acb.dac )
    sample_out_spi( &sample );


  debug_pins_low( DEBUG_P1 );
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
}






/****************************** End Of Module *******************************/
