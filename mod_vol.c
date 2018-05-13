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
#include <stdlib.h>
#include "string.h"
#include "audio.h"
#include "buffer.h"
#include "hid.h"
#include "lcd1602.h"

/*****************************    Defines    *******************************/
#define   NUM_PARAMETERS    1

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
uint8_t param_vol = 100;

/*****************************   Functions   *******************************/
void mod_vol_effekt( fp_sample_t *in, fp_sample_t *out)
{
  out->left_fp32 = (in->left_fp32 * ((float)param_vol/100));
  out->right_fp32 = (in->right_fp32 * ((float)param_vol/100));
}

void param_vol_get( void *param )
{
  param = &param_vol;
}

void param_vol_set( void *param )
{
  uint8_t vol = *(uint8_t*)param;

  if( vol > 200 )
    param_vol = 200;
  else
    param_vol = vol;
}

void mod_vol_init( void )
{
  param_vol = 100;
}

BOOLEAN mod_vol_setvol()
{
  BOOLEAN ret = TRUE;
  struct hid_msg msg;
  char buffer[3];

  lcd_clear();
  lcd_write("Master vol. %");
  lcd_set_cursor(0, 1);
  lcd_write("(0-200):");
  lcd_write( itoa( param_vol , buffer, 10 ) );

  if(hid_get(&msg))
  {
    if (msg.ch)
      lcd_write_char(msg.ch);
    if (msg.ch == ASCII_HASH)
      ret=FALSE;
    if( msg.function == HID_FUNC_DIGILEFT )
      param_vol = param_vol > 5 ? (param_vol - 5) : 0 ;
    if( msg.function == HID_FUNC_DIGIRIGHT )
      param_vol = param_vol < 200 ? (param_vol + 5) : 200 ;
    if( msg.function == HID_FUNC_DIGIP2 && msg.event == HID_EVENT_CLK)
      ret=FALSE;

  }
  return(ret);
}


/****************************** End Of Module *******************************/
