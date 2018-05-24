/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: hardware.h
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION:
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * --------------------
 * Mar 16, 2018	jorn    Module created.
 *
 *****************************************************************************/

#ifndef HARDWARE_H_
#define HARDWARE_H_

/***************************** Include files *******************************/
#include "emp_type.h"

/*****************************    Defines    *******************************/
#define LED_OFF           0b000
#define LED_COLOR_GREEN   0b001
#define LED_COLOR_BLUE    0b010
#define LED_COLOR_CYAN    0b011
#define LED_COLOR_RED     0b100
#define LED_COLOR_YELLOW  0b101
#define LED_COLOR_MAGENTA 0b110
#define LED_COLOR_WHITE   0b111

#define EMP_LED_RED       0b011
#define EMP_LED_YELLOW    0b101
#define EMP_LED_GREEN     0b110
#define EMP_LED_ALL       0b000

/********************** External declaration of Variables ******************/
typedef struct {
  INT16U left;
  INT16U right;
} sample_t ;

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/
INT8U is_sw1_pressed(void);
/*****************************************************************************
 *   Input    : -
 *   Output   : INT8U as Bool
 *   Function : Returns !0 if sw1 is pressed
 ******************************************************************************/

INT8U is_sw2_pressed(void);
/*****************************************************************************
 *   Input    : -
 *   Output   : INT8U
 *   Function : Returns !0 if sw2 is pressed
 ******************************************************************************/

void emp_set_led(INT8U led);
/*****************************************************************************
 *   Input    : 3 bit LED pattern
 *   Output   : -
 *   Function : Sets the EMP board leds
 ******************************************************************************/

void emp_clear_leds();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Clear all the EMP board leds
 ******************************************************************************/

void emp_toggle_status_led();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Toggles the status led
 ******************************************************************************/

void delay_us(INT32U time);
/*****************************************************************************
 *   Input    : Delay timer in approx microseconds
 *   Output   : -
 *   Function : delays for approx time in microseconds
 ******************************************************************************/

void sample_int_clear();
/*****************************************************************************
 *    Input    : -
 *    Output   : -
 *    Function : Clears the sample handler interrupt flag
 *******************************************************************************/

void sample_out_spi(sample_t *sample);
/*****************************************************************************
 *    Input    : Pointer to output sample of type sample_t
 *    Output   : -
 *    Function : Send sample to DAC via SPI
 *******************************************************************************/

void sample_out_pwm(sample_t *sample);
/*****************************************************************************
 *    Input    : Pointer to output sample of type sample_t
 *    Output   : -
 *    Function : Send sample to PWM
 *******************************************************************************/

void sample_in(sample_t *sample);
/*****************************************************************************
 *    Input    : Pointer to output sample of type sample_t
 *    Output   : -
 *    Function : Get sample from ADC
 *******************************************************************************/

INT8U is_digi_p2_pressed(void);
/*****************************************************************************
 *    Input    : -
 *    Output   : Boolean value
 *    Function : Check if digiswitch p2 is pressed
 *******************************************************************************/

INT8U is_digi_A(void);
/*****************************************************************************
 *    Input    : -
 *    Output   : Boolean value
 *    Function : Check if digiswitch function A is closed
 *******************************************************************************/

INT8U is_digi_B(void);
/*****************************************************************************
 *    Input    : -
 *    Output   : Boolean value
 *    Function : Check if digiswitch function B is closed
 *******************************************************************************/

void emp_toggle_led(INT8U led);
/*****************************************************************************
 *    Input    : 3 bit LED pattern
 *    Output   : -
 *    Function : Toggle EMP
 *******************************************************************************/

void hardware_init(INT32U sample_freq);
/*****************************************************************************
 *   Input    : Used Fs frequency
 *   Output   : -
 *   Function : Task to blink the EMP status led in 200ms interval
 ******************************************************************************/

/****************************** End Of Module *******************************/
#endif /* DEBUG_H_ */
