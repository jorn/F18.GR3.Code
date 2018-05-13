/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: hid.h
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION:
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * Apr 18, 2018	jorn    Module created.
 *
 *****************************************************************************/

#ifndef HID_H_
#define HID_H_

/***************************** Include files *******************************/
#include "emp_type.h"
#include "FreeRTOS.h"
#include "queue.h"

/*****************************    Defines    *******************************/
#define     ASCII_ESC           0x1B
#define     ASCII_DC1           0x11
#define     ASCII_DC2           0x12
#define     ASCII_DC3           0x13
#define     ASCII_CRETURN       0x0D
#define     ASCII_NEWLINE       0x0A
#define     ASCII_SPACE         0x20
#define     ASCII_HASH          0x23
#define     ASCII_ASTERISK      0x2A
#define     ASCII_0             0x30
#define     ASCII_9             0x39

#define     HID_FUNC_DIGI1      0x01
#define     HID_FUNC_DIGI2      0x02
#define     HID_FUNC_DIGIP2     0x03
#define     HID_FUNC_DIGILEFT   0x04
#define     HID_FUNC_DIGIRIGHT  0x05
#define     HID_FUNC_SW1        0x06
#define     HID_FUNC_SW2        0x07

#define     HID_EVENT_CLK       0x01
#define     HID_EVENT_DBL       0x02
#define     HID_EVENT_LONG      0x03


/********************** External declaration of Variables ******************/
typedef struct hid_msg {
  uint8_t   ch;
  uint8_t   function;
  uint8_t   event;
} hid_msg_t;
/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/
BaseType_t hid_init();
BaseType_t hid_get(hid_msg_t *msg );


/****************************** End Of Module *******************************/
#endif /* HID_H_ */
