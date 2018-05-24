/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: global.h
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION: Global definition file
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * --------------------
 * 16. mar. 2018	jorn    Module created.
 *
 *****************************************************************************/

#ifndef GLOBAL_H_
#define GLOBAL_H_

/***************************** Include files *******************************/

/*****************************    Defines    *******************************/
#define CPU_F                 80000000      // Current MCU clock speed
#define MS_PER_TICK           1             // Millisec per Ticks
#define CPU_MULTIPLEX         80            // CPU cycles per microsec

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define IDLE_PRIO 0
#define LOW_PRIO  1
#define MED_PRIO  2
#define HIGH_PRIO 3

// Bits
#define BIT_0     0x01
#define BIT_1     0x02
#define BIT_2     0x04
#define BIT_3     0x08
#define BIT_4     0x10
#define BIT_5     0x20
#define BIT_6     0x40
#define BIT_7     0x80

/********************* Macros **********************************************/
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

/****************************** End Of Module *******************************/
#endif /* GLOBAL_H_ */
