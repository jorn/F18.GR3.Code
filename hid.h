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

/********************** External declaration of Variables ******************/
QueueHandle_t xHIDQueue;

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/
BaseType_t hid_init();


/****************************** End Of Module *******************************/
#endif /* HID_H_ */
