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
#include "hid.h"
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
#include "semphr.h"
#include "task.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
QueueHandle_t xHIDQueue;
SemaphoreHandle_t xHIDQueueSem = NULL;

/*****************************   Functions   *******************************/
BaseType_t hid_get(hid_msg_t *msg)
{
  BaseType_t ret = pdFALSE;

  if (xHIDQueue != 0)
  {
    if ( xSemaphoreTake( xHIDQueueSem, 0) == pdTRUE)
    {
      ret = xQueueReceive(xHIDQueue, msg, 0);
      xSemaphoreGive(xHIDQueueSem);
    }
  }
  return (ret);
}

void digiswitch_handler(void)
{
  /* We have not woken a task at the start of the ISR. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  static uint32_t last_ticks = 0;
  uint8_t func = 0;
  struct hid_msg msg;

  if (ticks() - last_ticks >= 150)
  {
    // if A,B same -> CCW else CW
    func = is_digi_A() == is_digi_B() ? HID_FUNC_DIGILEFT : HID_FUNC_DIGIRIGHT;

    if (func)
    {
      msg.ch = 0;
      msg.event = 0;
      msg.function = func;
      xQueueSendToBackFromISR(xHIDQueue, &msg, &xHigherPriorityTaskWoken);
    }


  }
  last_ticks = ticks();

  // Clear int. for PA5
  bit_set(GPIO_PORTA_ICR_R, BIT_5);

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void keypad_handler(void)
{
  /* We have not woken a task at the start of the ISR. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // clear all keypad interrupts
  bit_set(GPIO_PORTE_ICR_R, BIT_0 | BIT_1 | BIT_2 | BIT_3);

  static uint32_t last_ticks = 0;
  const uint8_t keypad_acsii[4][3] = { "#0*", "987", "654", "321" };

  struct hid_msg msg;

  if (ticks() - last_ticks > 200)
  {
    uint8_t ch = 0;
    uint8_t x = 0;
    // get int. mask from PORTE
    uint8_t mask = GPIO_PORTE_DATA_R & 0b1111;

    bit_clear(GPIO_PORTA_DATA_R, 0b11100);
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");
    __asm("nop");__asm("nop");__asm("nop");__asm("nop");

    for(x=0; x<3; x++)
    {
      bit_set(GPIO_PORTA_DATA_R, 1 << (x + 2));
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");
      __asm("nop");__asm("nop");__asm("nop");__asm("nop");

      if ( !ch && (GPIO_PORTE_DATA_R & mask)  )
      {
        switch (mask)
        {
          case 0x01:
            ch = (uint8_t) keypad_acsii[0][x];
            break;
          case 0x02:
            ch = (uint8_t) keypad_acsii[1][x];
            break;
          case 0x04:
            ch = (uint8_t) keypad_acsii[2][x];
            break;
          case 0x08:
            ch = (uint8_t) keypad_acsii[3][x];
            break;
        }
      }
    }

    if (ch)
    {
      msg.ch = ch;
      msg.event = 0;
      msg.function = 0;
      xQueueSendToBackFromISR(xHIDQueue, &msg, &xHigherPriorityTaskWoken);
    }
  }
  last_ticks = ticks();



  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void digi_p2_task(void *pvParameters)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  uint32_t timer = ticks();
  uint32_t last_timer = 0;
  uint8_t event = 0;

  struct hid_msg msg;

  static enum states
  {
    IDLE, FIRST_PRESS, FIRST_RELEASE, SECOND_PRESS, LONG_PRESS
  } state = IDLE;

  while (1)
  {
    timer = ticks();
    event = 0;

    switch (state)
    {
      case IDLE:
        if (is_digi_p2_pressed())       // if button pushed
        {
          state = FIRST_PRESS;
          last_timer = timer;
        }
        break;
      case FIRST_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_digi_p2_pressed())    // if button released
          {
            state = FIRST_RELEASE;
            last_timer = timer;
          }
        }
        break;
      case FIRST_RELEASE:
        if (timer >= (last_timer + 100))          // if timeout
        {
          state = IDLE;
          event = HID_EVENT_CLK;
        }
        else
        {
          if (is_digi_p2_pressed())     // if button pressed
          {
            state = SECOND_PRESS;
            last_timer = timer;
          }
        }
        break;
      case SECOND_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_digi_p2_pressed())                    // if button released
          {
            state = IDLE;
            event = HID_EVENT_DBL;
          }
        }
        break;
      case LONG_PRESS:
        if (!is_digi_p2_pressed())                 // if button released
          state = IDLE;
        break;
      default:
        break;
    }

    if (event)
    {
      if ( xSemaphoreTake( xHIDQueueSem, 0) == pdTRUE)
      {
        msg.ch = 0;
        msg.event = event;
        msg.function = HID_FUNC_DIGIP2;
        xQueueSendToBack(xHIDQueue, &msg, 0);
        xSemaphoreGive(xHIDQueueSem);
      }
    }

    vTaskDelay(10);
  }
}

void sw1_task(void *pvParameters)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  uint32_t timer = ticks();
  uint32_t last_timer = 0;
  uint8_t event = 0;

  struct hid_msg msg;

  static enum states
  {
    IDLE, FIRST_PRESS, FIRST_RELEASE, SECOND_PRESS, LONG_PRESS
  } state = IDLE;

  while (1)
  {
    timer = ticks();
    event = 0;

    switch (state)
    {
      case IDLE:
        if (is_sw1_pressed())       // if button pushed
        {
          state = FIRST_PRESS;
          last_timer = timer;
        }
        break;
      case FIRST_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_sw1_pressed())    // if button released
          {
            state = FIRST_RELEASE;
            last_timer = timer;
          }
        }
        break;
      case FIRST_RELEASE:
        if (timer >= (last_timer + 100))          // if timeout
        {
          state = IDLE;
          event = HID_EVENT_CLK;
        }
        else
        {
          if (is_sw1_pressed())     // if button pressed
          {
            state = SECOND_PRESS;
            last_timer = timer;
          }
        }
        break;
      case SECOND_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_sw1_pressed())                    // if button released
          {
            state = IDLE;
            event = HID_EVENT_DBL;
          }
        }
        break;
      case LONG_PRESS:
        if (!is_sw1_pressed())                 // if button released
          state = IDLE;
        break;
      default:
        break;
    }

    if (event)
    {
      if ( xSemaphoreTake( xHIDQueueSem, 0) == pdTRUE)
      {
        msg.ch = 0;
        msg.event = event;
        msg.function = HID_FUNC_SW1;
        xQueueSendToBack(xHIDQueue, &msg, 0);
        xSemaphoreGive(xHIDQueueSem);
      }
    }
    vTaskDelay(10);
  }
}

void sw2_task(void *pvParameters)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  uint32_t timer = ticks();
  uint32_t last_timer = 0;
  uint8_t event = 0;

  struct hid_msg msg;

  static enum states
  {
    IDLE, FIRST_PRESS, FIRST_RELEASE, SECOND_PRESS, LONG_PRESS
  } state = IDLE;

  while (1)
  {
    timer = ticks();
    event = 0;

    switch (state)
    {
      case IDLE:
        if (is_sw2_pressed())       // if button pushed
        {
          state = FIRST_PRESS;
          last_timer = timer;
        }
        break;
      case FIRST_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_sw2_pressed())    // if button released
          {
            state = FIRST_RELEASE;
            last_timer = timer;
          }
        }
        break;
      case FIRST_RELEASE:
        if (timer >= (last_timer + 100))          // if timeout
        {
          state = IDLE;
          event = HID_EVENT_CLK;
        }
        else
        {
          if (is_sw2_pressed())     // if button pressed
          {
            state = SECOND_PRESS;
            last_timer = timer;
          }
        }
        break;
      case SECOND_PRESS:
        if (timer >= (last_timer + 2000))          // if timeout
        {
          state = LONG_PRESS;
          event = HID_EVENT_LONG;
        }
        else
        {
          if (!is_sw2_pressed())                    // if button released
          {
            state = IDLE;
            event = HID_EVENT_DBL;
          }
        }
        break;
      case LONG_PRESS:
        if (!is_sw2_pressed())                 // if button released
          state = IDLE;
        break;
      default:
        break;
    }

    if (event)
    {
      if ( xSemaphoreTake( xHIDQueueSem, 0) == pdTRUE)
      {
        msg.ch = 0;
        msg.event = event;
        msg.function = HID_FUNC_SW2;
        xQueueSendToBack(xHIDQueue, &msg, 0);
        xSemaphoreGive(xHIDQueueSem);
      }
    }
    vTaskDelay(10);
  }
}

BaseType_t hid_init()
{
  portBASE_TYPE return_value = pdTRUE;

  xHIDQueue = xQueueCreate(20, sizeof(struct hid_msg));

  xHIDQueueSem = xSemaphoreCreateMutex();

  return_value &= xTaskCreate(digi_p2_task, "HID DIGIP2",
  USERTASK_STACK_SIZE,
                              NULL, LOW_PRIO, NULL);
  return_value &= xTaskCreate(sw1_task, "HID SW1",
  USERTASK_STACK_SIZE,
                              NULL, LOW_PRIO, NULL);
  return_value &= xTaskCreate(sw2_task, "HID SW2",
  USERTASK_STACK_SIZE,
                              NULL, LOW_PRIO, NULL);

  if (xHIDQueue == NULL)
    return_value &= pdFALSE;

  return (return_value);
}

/****************************** End Of Module *******************************/
