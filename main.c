/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: main.c
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION:
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * 6. mar. 2018  jorn    Created
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "systick.h"
#include "hardware.h"
#include "FreeRTOS.h"
#include "task.h"
#include "global.h"
#include "hid.h"
#include "lcd1602.h"
#include "debug.h"
#include "queue.h"
#include "audio.h"

/*****************************    Defines    *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/

int putChar()
/*****************************************************************************
*   Input    :  -
*   Output   :  Result
*   Function :  putChar for FreeRTOS debug functionality.
*****************************************************************************/
{
  return(0);
}

void emp_board_alive(void *pvParameters)
/*****************************************************************************
*   Input    :  Process Parameters
*   Output   :  -
*   Function :  EMP board alive led task
*****************************************************************************/
{
  while(1)
  {
    emp_toggle_status_led();
    vTaskDelay(500);
  }
}


void status_display(void *pvParameters)
{
  while (1)
  {
    uint8_t value;

    if (xHIDQueue != 0)
    {
      if (xQueueReceive(xHIDQueue, &value, (TickType_t) 1))
      {
        if (value == 0x1b)
        {
          xQueueReceive(xHIDQueue, &value, (TickType_t) 1);

          switch (value)
          {
            case 0x10:
              lcd_write_char(0x3C);
              break;
            case 0x01:
              lcd_write_char(0x3E);
              break;
          }
        }
        else
          lcd_write_char(value);

      }
    }
    vTaskDelay(100);
  }
}


int main(void)
{
  portBASE_TYPE return_value = pdTRUE;

  hardware_init(44100);      // Init the hardware to Fs=44.100 Hz

  emp_clear_leds();

  audio_init();

  return_value &= xTaskCreate( emp_board_alive, "EMP Board Alive",
                               USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);


  return_value &= xTaskCreate( status_display, "Status Display",
                               USERTASK_STACK_SIZE, NULL, MED_PRIO, NULL);

  lcd_init();           // Init the lcd_driver

  return_value &= hid_init();

  if (return_value != pdTRUE)
  {
    GPIO_PORTD_DATA_R &= 0xBF;  // Turn on status LED.
    while(1);  // cold not create one or more tasks.
  }

  // Start the scheduler.
  // --------------------
  vTaskStartScheduler();

  // Will only get here, if there was insufficient memory.
  // -----------------------------------------------------
  return 1;
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}





/****************************** End Of Module *******************************/
