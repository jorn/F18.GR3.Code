/*****************************************************************************
* University of Southern Denmark
* Embedded C Programming (ECP)
*
* MODULENAME.: lcd1602.h
*
* PROJECT....: F18.GR3.Code
*
* DESCRIPTION: EMP Lcd library
*
* Change Log:
******************************************************************************
* Date    Id    Change
* YYMMDD
* --------------------
* 180419  JJA   Module created.
*
*****************************************************************************/

#ifndef EMP_LCD1602_
#define EMP_LCD1602_

/***************************** Include files *******************************/
#include "emp_type.h"
#include "FreeRTOS.h"
#include "task.h"

/*****************************    Defines    *******************************/
#define     LCD_CHARS               16
#define     LCD_LINES               2
#define     LCD_BUF_SIZE            LCD_CHARS*LCD_LINES

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/
BaseType_t lcd_init();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Initialize the LCD driver
 *******************************************************************************/

void lcd_clear();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Clear the LCD
 *******************************************************************************/

void lcd_write(char *str);
/*****************************************************************************
 *    Input    : Pointer to string to write
 *    Output   : -
 *    Function : Write string to LCD
 *******************************************************************************/

void lcd_write_char(char ch);
/*****************************************************************************
 *    Input    : Char to write
 *    Output   : -
 *    Function : Write char to LCD
 *******************************************************************************/

void lcd_set_cursor(INT8U x, INT8U y);
/*****************************************************************************
 *    Input    : Cursor position x and y
 *    Output   : -
 *    Function : Set the cursor position on the LCD
 *******************************************************************************/

void lcd_direct_write_instruction(INT8U byte);
/*****************************************************************************
 *    Input    : Data to write
 *    Output   : -
 *    Function : Write Instruction directly to LCD
 *******************************************************************************/

void lcd_direct_write_data(INT8U byte);
/*****************************************************************************
 *    Input    : Data to write
 *    Output   : -
 *    Function : Write Data directly to LCD
 *******************************************************************************/


/****************************** End Of Module *******************************/
#endif
