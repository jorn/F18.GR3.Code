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
#include <stdlib.h>
#include "string.h"
#include "audio.h"
#include "buffer.h"
#include "rebuf.h"
#include "hid.h"
#include "lcd1602.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
uint8_t param_gain;
uint8_t param_delay;

/*****************************   Functions   *******************************/
void mod_reverb_effekt( fp_sample_t *in, fp_sample_t *out)
{

  float in_gain = -((float)param_gain/100);
  float fb_gain = in_gain;
  uint16_t delay = param_delay*44;

  //const float in_gain = -0.45;
  //const float fb_gain = -0.50;
  //const uint16_t delay = 2000;

  fp_sample_t fp_sample;
  re_buffer_get(&fp_sample);

  fp_sample.left_fp32 = ((in->left_fp32 * in_gain) + ((in->left_fp32 + fp_sample.left_fp32) * fb_gain));
  fp_sample.right_fp32 = ((in->right_fp32 * in_gain) + ((in->left_fp32 + fp_sample.right_fp32) * fb_gain));

  re_buffer_put_z(&fp_sample, delay);

  out->left_fp32 = in->left_fp32;
  out->right_fp32 = in->right_fp32;
}

void mod_reverb_init( void )
{
  param_delay = 22;
  param_gain = 50;
}

BOOLEAN mod_reverb_setgain()
{
  BOOLEAN ret = TRUE;
  struct hid_msg msg;
  char buffer[3];

  lcd_clear();
  lcd_write("Reverb gain %");
  lcd_set_cursor(0, 1);
  lcd_write("(0-200):");
  lcd_write( itoa( param_gain , buffer, 10 ) );

  if(hid_get(&msg))
  {
    if (msg.ch)
      lcd_write_char(msg.ch);
    if (msg.ch == ASCII_HASH)
      ret=FALSE;
    if( msg.function == HID_FUNC_DIGILEFT )
      param_gain = param_gain > 5 ? (param_gain - 5) : 0 ;
    if( msg.function == HID_FUNC_DIGIRIGHT )
      param_gain = param_gain < 200 ? (param_gain + 5) : 200 ;
    if( msg.function == HID_FUNC_DIGIP2 && msg.event == HID_EVENT_CLK)
      ret=FALSE;
  }
  return(ret);
}

BOOLEAN mod_reverb_setdelay()
{
  BOOLEAN ret = TRUE;
  struct hid_msg msg;
  char buffer[3];

  lcd_clear();
  lcd_write("Reverb delay (ms)");
  lcd_set_cursor(0, 1);
  lcd_write("(2-44):");
  lcd_write( itoa( param_delay , buffer, 10 ) );

  if(hid_get(&msg))
  {
    if (msg.ch)
      lcd_write_char(msg.ch);
    if (msg.ch == ASCII_HASH)
      ret=FALSE;
    if( msg.function == HID_FUNC_DIGILEFT )
      param_delay = param_delay > 4 ? (param_delay - 2) : 2 ;
    if( msg.function == HID_FUNC_DIGIRIGHT )
      param_delay = param_delay < 42 ? (param_delay + 2) : 44 ;
    if( msg.function == HID_FUNC_DIGIP2 && msg.event == HID_EVENT_CLK)
      ret=FALSE;

  }
  return(ret);
}


/****************************** End Of Module *******************************/
