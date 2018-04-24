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

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
extern QueueHandle_t xHIDQueue;


void digiswitch_handler(void)
{
  static INT8U state[2][2] = {{ 0, 0 },{ 0, 0 }};
  const INT8U A = 0;                    // PA5
  const INT8U B = 1;                    // PA6

  /* We have not woken a task at the start of the ISR. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int8_t out = 0;

  if( GPIO_PORTA_RIS_R & BIT_5 )
  {
    bit_set( GPIO_PORTA_ICR_R, BIT_5);      // Clear int. for PA5
    state[A][0] = 1;
  }

  if( GPIO_PORTA_RIS_R & BIT_6 )
  {
    bit_set( GPIO_PORTA_ICR_R, BIT_6);      // Clear int. for PA6
    state[B][0] = 1;
  }

  // Switch logic
  if( state[B][0] && state[A][1] )
  {  // CCW ??
    out = 1;
    emp_clear_leds();
    emp_set_led( EMP_LED_RED );
  }

  if( state[A][0] && state[B][1] )
  {   // CW ??
    out = 2;
    emp_clear_leds();
    emp_set_led( EMP_LED_GREEN );
  }

//  if(out)
//    xQueueSendToBackFromISR( xHIDQueue, &out, &xHigherPriorityTaskWoken );


  // Store switch history
  state[A][1] = state[A][0];
  state[A][0] = 0;
  state[B][1] = state[B][0];
  state[B][0] = 0;

  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}


/*****************************   Functions   *******************************/
void digiswitch_task(void *pvParameters)
{
  static INT8U AB[2][2] = { {0,0},{0,0} };     // AB[N]
  static INT8U YY[2]    = { 0,0 };             // YY[AB]
  //static INT16U deg     = 0;

  const INT8U A = 0;
  const INT8U B = 1;

  //const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  while(1)
  {
    // Check Digi P2
    if( !( GPIO_PORTA_DATA_R & 0b10000000) )
    {
      // Digiswitch click
      xQueueSendToBack( xHIDQueue, "c", 10 );
    }

    // Read inputs
    AB[1][A] = !(GPIO_PORTA_DATA_R & 0b00100000);    // Read Digi A
    AB[1][B] = !(GPIO_PORTA_DATA_R & 0b01000000);    // Read Digi B

    if( (AB[1][A] != AB[0][A]) || (AB[1][B] != AB[0][B]) )
    {
      YY[A] = AB[1][A] != AB[0][A];
      YY[B] = AB[1][B] != AB[0][B];

      if( AB[1][A] == AB[1][B] )
      {
        if( !YY[A] && YY[B] )
        {
          // right toggle
          xQueueSendToBack( xHIDQueue, "R", 10 );

          //emp_toggle_led(LED_YELLOW);
          //deg += 6;
          //if(deg==360)
          //  deg = 0;
        }
        else if ( YY[A] && !YY[B] )
        {
          xQueueSendToBack( xHIDQueue, "L", 10 );
          //emp_toggle_led(LED_GREEN);
          //if(deg == 0)
          //  deg = 360-6;
          //else
          //  deg -= 6;
        }
        else
        {
          // spike
        }
      }
      else
      {
        if( YY[A] && !YY[B] )
        {
          xQueueSendToBack( xHIDQueue, "R", 10 );
          //emp_toggle_led(LED_YELLOW);
          //deg += 6;
          //if(deg==360)
          //  deg = 0;
        }
        else if ( !YY[A] && YY[B] )
        {
          xQueueSendToBack( xHIDQueue, "L", 10 );
          //emp_toggle_led(LED_GREEN);
          //if(deg == 0)
          //  deg = 360-6;
          //else
          //  deg -= 6;
        }
        else
        {
          // spike
        }
      }

      AB[0][A] = AB[1][A];
      AB[0][B] = AB[1][B];
    }

    //lcd_set_cursor(0, 0);
    //lcd_write("Angel :    ");
    //lcd_set_cursor(8, 0);
    //char b[3];
    //lcd_write( itoa(deg, b) );

    //vTaskDelay(xDelay);
  }
}

BaseType_t hid_init()
{
  portBASE_TYPE return_value = pdTRUE;

  xHIDQueue = xQueueCreate( 5, sizeof( int8_t ) );

  if( xHIDQueue != NULL )
  {

    /*
  return_value &= xTaskCreate( digiswitch_task, "HID Digiswitch",
                               USERTASK_STACK_SIZE, NULL, LOW_PRIO , NULL);
*/
  }
  else
    return_value = pdFALSE;

  return( return_value );
}




/****************************** End Of Module *******************************/
