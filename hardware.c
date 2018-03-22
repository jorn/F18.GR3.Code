/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: hardware.c
 *
 * PROJECT....: F18.GR3
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * Mar 16, 2018  jorn    Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "global.h"
#include "emp_type.h"
#include "tm4c123gh6pm.h"
#include "systick.h"
#include "hardware.h"

/*****************************    Defines    *******************************/
#define TIM_2_SEC           2000
#define TIM_100_MSEC         100
#define TIM_200_MSEC         200

#define STATUS_LED_TIMER    TIM_200_MSEC

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/

/*****************************   Functions   *******************************/
INT8U is_sw1_pressed(void)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  return ((GPIO_PORTF_DATA_R & 0x10) ? 0 : 1);
}

INT8U is_sw2_pressed(void)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  return ((GPIO_PORTF_DATA_R & 0x01) ? 0 : 1);
}


void emp_set_led(INT8U led)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  // Set bit pattern bit2-bit0 on for led_color on PF1-PF3
  if (led & 0x01)
    GPIO_PORTF_DATA_R |= 0x08;
  else
    GPIO_PORTF_DATA_R &= ~0x08;

  if (led & 0x02)
    GPIO_PORTF_DATA_R |= 0x04;
  else
    GPIO_PORTF_DATA_R &= ~0x04;

  if (led & 0x04)
    GPIO_PORTF_DATA_R |= 0x02;
  else
    GPIO_PORTF_DATA_R &= ~0x02;
}

void emp_toggle_status_led()
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  GPIO_PORTD_DATA_R ^= 0x40;
}

void emp_clear_leds()
{
  emp_set_led( ~EMP_LED_ALL );
}

void init_tiva_board()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the Tiva board
 ******************************************************************************/
{
  // Set GPIO'S on Run Mode Clock Gating Control Register on PORTF & PORTD
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3;

  // Unlock PORTF
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R = 0xFF;

  // GPIO Direction (GPIODR)
  GPIO_PORTF_DIR_R = 0x0E;    // 0b00001110
  // Set the direction as output (PD6).
  GPIO_PORTD_DIR_R = 0x40;    // 0b01000000

  // GPIO Digital Enable (GPIODEN)
  GPIO_PORTF_DEN_R = 0x1F;    // 0b00011111
  // Enable the GPIO pins for digital function (PD6).
  GPIO_PORTD_DEN_R = 0x40;    // 0b01000000

  // GPIO PullUp Resistor (GPIOPUR)
  GPIO_PORTF_PUR_R = 0x11;    // 0b00010001

}

void delay_us(INT32U time)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  //Make sure Timer2A is stopped
  bit_clear(TIMER2_CTL_R, TIMER_CTL_TAEN);

  //Set the start value of timer2 A
  TIMER2_TAILR_R = time * CPU_MULTIPLEX;

  // Start timer2 A
  bit_set(TIMER2_CTL_R, TIMER_CTL_TASTALL);  // stall timer2 in debug mode
  bit_set(TIMER2_CTL_R, TIMER_CTL_TAEN);

  // poll for time-out on timer2 A
  while(!(TIMER2_RIS_R & TIMER_RIS_TATORIS));

  // clear flag
  bit_set(TIMER2_ICR_R, TIMER_ICR_TATOCINT);
}

void delay_init()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the the microsecond delay
 ******************************************************************************/{
  // Activate Timer2 Clock
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;

  // Disable timer2 A before changing any value
  bit_clear(TIMER2_CTL_R, TIMER_CTL_TAEN);

  // 32-bit timer for Timer2 A.
  TIMER2_CFG_R = 0x00;

  // One-Shot Timer mode on Timer2 A
  TIMER2_TAMR_R = 0x01;
}

void set_80Mhz_clock()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Set the processor frequency to 80Mhz
 ******************************************************************************/
{
  INT32U reg;

  // Enable the use of RCC2
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;     // Set USERCC2

  // Bypass PLL ans system clock divider
  SYSCTL_RCC_R |= SYSCTL_RCC_BYPASS;        // Set Bypass on RCC
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;     // Set Bypass on RCC2
  SYSCTL_RCC_R &= ~SYSCTL_RCC_USESYSDIV;    // Clear USESYS

  // Set XTAL to 16Mhz
  reg = SYSCTL_RCC_R;
  reg &= ~(0b11111<<6);
  reg |= SYSCTL_RCC_XTAL_16MHZ;
  SYSCTL_RCC_R = reg;

  // Set Oscillator Source OSCSRC
  SYSCTL_RCC2_R &= ~0b1110000;  // Set MOSC

  // Clear the PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;


  // Set DIV400 bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;

  // Set system divider SYSDIV to 0x2 and SSYSDIV2LSB to 0
  reg = SYSCTL_RCC2_R;
  reg &= ~(0b1111111<<22);
  reg |= (0b100<<22);
  SYSCTL_RCC2_R = reg;

  // Enable the new divider by setting USESYS
  SYSCTL_RCC_R |= SYSCTL_RCC_USESYSDIV;    // Set USESYS

  // Wait for PLL lock, value=1 when PLL is locked
  while( !(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS) );

  // Enable PLL
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;  // Clear Bypass2

}

void hardware_init()
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  // disable global interrupt
  disable_global_int();

  set_80Mhz_clock();

  delay_init();

  // Initialize the Tiva board
  init_tiva_board();

  systick_init();

  // Enable global interrupt
  enable_global_int();
}

/****************************** End Of Module *******************************/
