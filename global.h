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
#define CPU_F                 16000000      // Current MCU clock speed
#define MS_PER_TICK           1             // Millisec per Ticks
#define CPU_MULTIPLEX         80            // CPU cycles per microsec

// UART
#define UART_BAUDRATE         19200         // UART baudrate
#define UART_DATABITS         8             // UART databits
#define UART_STOPBITS         1             // UART stopbits
#define UART_PARITY           'n'           // UART no-parity

// Files
#define   COM1                0             // UART file to use

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
