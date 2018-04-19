/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: debug.h
 *
 * PROJECT....: EQ_ONE
 *
 * DESCRIPTION:
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * 16. apr. 2017	jorn    Module created.
 *
 *****************************************************************************/

#ifndef DEBUG_H_
#define DEBUG_H_

/***************************** Include files *******************************/
#include "emp_type.h"

/*****************************    Defines    *******************************/
// GPIO port used for debug
#define       DEBUG_PORT      GPIO_PORTB_DATA_R

// Debug PIN-MAP on DEBUG_PORT
#define       DEBUG_P1        0x1
#define       DEBUG_P2        0x2
#define       DEBUG_P3        0x8



/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/
void debug_pins_high(INT8U pins);
/*****************************************************************************
 *   Input    : Pin bit mask
 *   Output   : -
 *   Function : Sets the selected pins bitmask high
 ******************************************************************************/

void debug_pins_low(INT8U pins);
/*****************************************************************************
 *   Input    : Pin bit mask
 *   Output   : -
 *   Function : Sets the selected pins bitmask low
 ******************************************************************************/

void debug_pins_toggle(INT8U pins);
/*****************************************************************************
 *   Input    : Pin bit mask
 *   Output   : -
 *   Function : Toggles the selected pins bitmask
 ******************************************************************************/

/****************************** End Of Module *******************************/
#endif /* DEBUG_H_ */
