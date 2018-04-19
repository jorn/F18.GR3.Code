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
static sample_t *sample;

/*****************************   Functions   *******************************/
void sample_handler( void )
{
  sample_int_clear();

  // Debug toggle debug pin 3
  debug_pins_toggle( DEBUG_P3 );

  sample_get( &sample );
  sample_put( &sample );
}




/****************************** End Of Module *******************************/
