/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: systick.c
 *
 * PROJECT....:
 *
 * DESCRIPTION: sysTick module
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * 16. mar. 2018  jorn    Module adopted from MOH systick
 *
 *****************************************************************************/

#ifndef _SYSTICK_H
#define _SYSTICK_H

/***************************** Include files *******************************/

/*****************************    Defines    *******************************/
#define CPU_F         16000000
#define MS_PER_TICK   1
/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/
void enable_global_int();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Enable global interrupt.
 ******************************************************************************/

void disable_global_int();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Disable global interrupt.
 ******************************************************************************/

void systick_init();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the systick interrupt.
 ******************************************************************************/

INT32U systick_touch();
/*****************************************************************************
 *   Input    : -
 *   Output   : systick value since last touch
 *   Function : Touch the systick timer
 ******************************************************************************/


/****************************** End Of Module *******************************/
#endif


