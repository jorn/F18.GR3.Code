/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: hid.c
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * Apr 17, 2018  jorn    Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "global.h"
#include "lcd1602.h"
#include "debug.h"
#include "hardware.h"
#include "systick.h"


/*****************************    Defines    *******************************/
#define     ESC      0x1B

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
extern QueueHandle_t xHIDQueue;

/*****************************   Functions   *******************************/
void digiswitch_handler(void)
{
  /* We have not woken a task at the start of the ISR. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t out = 0;
  const uint8_t esc = ESC;
  static uint32_t last_ticks = 0;

  if (ticks() - last_ticks >= 1)
  {
    // if A,B same -> CCW=0x10 else CW=0x01
    out = is_digi_A() == is_digi_B() ? 0x10 : 0x01;

    if (out)
    {
      xQueueSendToBackFromISR(xHIDQueue, &esc, &xHigherPriorityTaskWoken);
      xQueueSendToBackFromISR(xHIDQueue, &out, &xHigherPriorityTaskWoken);
    }

    last_ticks = ticks();
  }

  // Clear int. for PA5
  bit_set(GPIO_PORTA_ICR_R, BIT_5);

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void keypad_handler(void)
{
  /* We have not woken a task at the start of the ISR. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t out = 0, x=0;
  static uint32_t last_ticks = 0;
  const uint8_t keypad_acsii[4][3] = { "#0*", "987", "654", "321" };

  //bit_clear(GPIO_PORTE_IM_R, BIT_0 | BIT_1 | BIT_2 | BIT_3);

  debug_pins_high(DEBUG_P1);

  if (ticks() - last_ticks > 200)
  {
    bit_clear(GPIO_PORTA_DATA_R, 0b11000);
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    // get int. mask from PORTE
    uint8_t mask = GPIO_PORTE_RIS_R & 0b1111;

    while ( x < 3 )
    {
      if ( GPIO_PORTE_DATA_R & 0b1111)
      {
        switch ( mask )
        {
          case 0x01:
            out = (uint8_t) keypad_acsii[0][x];
            break;
          case 0x02:
            out = (uint8_t) keypad_acsii[1][x];
            break;
          case 0x04:
            out = (uint8_t) keypad_acsii[2][x];
            break;
          case 0x08:
            out = (uint8_t) keypad_acsii[3][x];
            break;
          default:
            out = 0x41;
        }
        x=3;
      }
      x++;
      bit_set(GPIO_PORTA_DATA_R, 1 << (x + 2));
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      //bit_clear(GPIO_PORTA_DATA_R, 0b11100);
    }

    bit_set(GPIO_PORTA_DATA_R, 0b11100);

    //if ( GPIO_PORTE_RIS_R && BIT_0 )

    if (out)
    {
      xQueueSendToBackFromISR(xHIDQueue, &out, &xHigherPriorityTaskWoken);
    }
    last_ticks = ticks();


  }
  debug_pins_low(DEBUG_P1);

  // clear all keypad interrupts
  bit_set(GPIO_PORTE_ICR_R, BIT_0 | BIT_1 | BIT_2 | BIT_3);
  //bit_set(GPIO_PORTE_IM_R, BIT_0 | BIT_1 | BIT_2 | BIT_3);

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

BaseType_t hid_init()
{
  portBASE_TYPE return_value = pdTRUE;

  xHIDQueue = xQueueCreate(20, sizeof(uint8_t));

  if (xHIDQueue == NULL)
    return_value = pdFALSE;

  return (return_value);
}

/****************************** End Of Module *******************************/
