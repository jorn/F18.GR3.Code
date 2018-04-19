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


#define onboard
/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
FP32 adc_pwm_ratio = 1;

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

void init_digiswitch()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the digiswitch
 ******************************************************************************/
{
  bit_set( SYSCTL_RCGCGPIO_R, SYSCTL_RCGCGPIO_R0 );   // PORTA

  // Setup DIGISwitch (PA5, PA6, PA7)
  bit_clear( GPIO_PORTA_DIR_R, 0b11100000 );    // Input
  bit_set  ( GPIO_PORTA_DEN_R, 0b11100000 );
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

void init_ADC( )
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the ADC
 ******************************************************************************/
{
  //  ANALOG_IN LEFT  -> PE4 (AIN9) -> (ADC1)
  //            RIGHT -> PE5 (AIN8) -> (ADC0)

  // Module Init
  // Enable ADC0 & ADC1 Clock
  SYSCTL_RCGCADC_R |= (SYSCTL_RCGCADC_R0 | SYSCTL_RCGCADC_R1);

  // Enable GPIO clock for port E
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOE;

  // Enable the alternate function on PE4 and PE5, high==AFSEL
  GPIO_PORTE_AFSEL_R |= (1<<4 & 1<<5) ;

  // Set inputs as analog on port E
  GPIO_PORTE_DEN_R &= ~(1<<4 & 1<<5);

  // Select analog mode AMSEL on PortE
  GPIO_PORTE_AMSEL_R |= (1<<4 & 1<<5);

  // Sample Sequencer Configuration

  // Disable ASEN3:0
  bit_clear( ADC0_ACTSS_R, 0b1111);
  bit_clear( ADC1_ACTSS_R, 0b1111);

  // Configure trigger event for SS ADCEMUX, set EM3 : Processor event (ADCPSSI)
  ADC0_EMUX_R &= 0;
  ADC1_EMUX_R &= 0;

  // Config input source for each sample i SS3, AIN8 & AIN9
  ADC0_SSMUX3_R = 8;
  ADC1_SSMUX3_R = 9;

  // Set Sample Sequence Control 3
  // 3 : TS0  - Temp Sensor
  // 2 : IE0  - Interrupt Enable
  // 1 : END0 - End of Sequence
  // 0 : D0   - Sample Diff input Select
  ADC0_SSCTL3_R = 0b0010;
  ADC1_SSCTL3_R = 0b0010;

  // Set ADC Interrupt Mask (if used)
  ADC0_IM_R = 0b0000;
  ADC1_IM_R = 0b0000;

  // Disable ASEN3:0
  bit_set( ADC0_ACTSS_R, 0b1111);
  bit_set( ADC1_ACTSS_R, 0b1111);
}

void init_PWM( INT16U cycles )
/*****************************************************************************
 *   Input    : Cycles matching the samplings frequency
 *   Output   : -
 *   Function : Initialize the PWM as audio output based on the cycles
 *              between each sample to PB2
 ******************************************************************************/
{
  // PB2 (T3CCP0)

  // Activate PORTB Clock
  bit_set( SYSCTL_RCGC2_R, SYSCTL_RCGC2_GPIOB );

  // Digital enable PB2
  bit_set( GPIO_PORTB_DEN_R, 1<<2 );

  // Enable the alternate function on PortB pin 2 PB2, high==AFSEL
  bit_set( GPIO_PORTB_AFSEL_R, 1<<2 );

  // GPIO portcontrol, activate T3CCP0 on pin PB2
  bit_clear( GPIO_PORTB_PCTL_R, GPIO_PCTL_PB2_M );
  bit_set( GPIO_PORTB_PCTL_R, GPIO_PCTL_PB2_T3CCP0 );

  // Activate Timer3 Clock
  bit_set( SYSCTL_RCGCTIMER_R, SYSCTL_RCGCTIMER_R3 );

  // Disable Timer 3
  bit_clear( TIMER3_CTL_R, TIMER_CTL_TAEN);

  // Select 16/32 bit mode for 32/64bit timer
  bit_set( TIMER3_CFG_R, TIMER_CFG_16_BIT );

  //In the GPTM Timer Mode (GPTMTnMR) register, set the
  // TnAMS bit to 0x1  - PWM mode enabled
  // TnCMR bit to 0x0  - Edge count mode
  // TnMR field to 0x2 - Periodic time mode
  bit_set( TIMER3_TAMR_R, TIMER_TAMR_TAAMS );
  bit_clear( TIMER3_TAMR_R , TIMER_TAMR_TACMR );
  bit_set( TIMER3_TAMR_R, TIMER_TAMR_TAMR_PERIOD );

  // Configure the output state of the PWM signal
  // (whether or not it is inverted) in the TnPWML field
  // of the GPTM Control (GPTMCTL) register.

  //bit_clear( TIMER3_CTL_R, TIMER_CTL_TAPWML ); // PWM signal is not inverted
  bit_set( TIMER3_CTL_R, TIMER_CTL_TAPWML ); // PWM signal IS inverted

  //  If a prescaler is to be used, write the prescale value to the
  //  GPTM Timer n Prescale Register (GPTMTnPR).

  // TIMER3_TAPR_R = 0;

  // Load the timer start value into the GPTM Timer n Interval Load
  // (GPTMTnILR) register.

  TIMER3_TAILR_R = cycles;

  //  Load the GPTM Timer n Match (GPTMTnMATCHR) register with
  //  the match value.
  TIMER3_TAMATCHR_R = 0;

  TIMER3_ICR_R |= TIMER_ICR_CAECINT;

  // Enable NVIC interrupt 35
  bit_set(NVIC_EN1_R, 1<<3 );

  bit_set( TIMER3_TAMR_R, TIMER_TAMR_TAPWMIE);
  bit_set( TIMER3_IMR_R, TIMER_IMR_CAEIM);

  // Enable Timer3 Interrupt on Timer A Falling Edge
  bit_clear( TIMER3_CTL_R , TIMER_CTL_TAEVENT_M);
  bit_set( TIMER3_CTL_R, TIMER_CTL_TAEVENT_NEG);



  // Timer A freezes counting while the processor is halted by the   debugger.
  bit_set( TIMER3_CTL_R, TIMER_CTL_TASTALL);


  // Enable Timer 3
  bit_set( TIMER3_CTL_R, TIMER_CTL_TAEN);
}

void enable_FPU()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Enable the floating point unit (FPU)
 ******************************************************************************/
{
  INT32U reg;

  reg = NVIC_CPAC_R;    // Coprocessor Access  Control (CPAC) register
  reg |= (0xF << 20);   // Set bits 20-23 to enable CP10 and CP11 coprocessors
  NVIC_CPAC_R = reg;

  __asm__("DSB");       // Data Synchronisation Barrier
  __asm__("ISB");       // Instruction Synchronisation Barrier
}

void delay_init()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the the microsecond delay
 ******************************************************************************/
{
  // Activate Timer2 Clock
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;

  // Disable timer2 A before changing any value
  bit_clear(TIMER2_CTL_R, TIMER_CTL_TAEN);

  // 32-bit timer for Timer2 A.
  TIMER2_CFG_R = 0x00;

  // One-Shot Timer mode on Timer2 A
  TIMER2_TAMR_R = 0x01;
}

void spi_init()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the SPI2
 ******************************************************************************/
{
  // Activate the SSI module clock for SSI2
  bit_set( SYSCTL_RCGCSSI_R, SYSCTL_RCGCSSI_R2 );

  // Activate GPIO port B click
  bit_set( SYSCTL_RCGCGPIO_R, SYSCTL_RCGCGPIO_R1);

  // Digital enable pins
  GPIO_PORTB_DEN_R |= (1<<4 | 1<<5 | 1<<6 | 1<<7);

  // Set direction to output for CLK, SS and TX pins
  GPIO_PORTB_DIR_R |= (1<<4 | 1<<5 | 1<<7);

  // GPIO PullUp Resistor (GPIOPUR)
  GPIO_PORTB_PUR_R |=  (1<<6);    // pull up on SSI2RX pin 4

  // Enable the alternate function on PB4, PB5, PB6 & PB6, high==AFSEL
  GPIO_PORTB_AFSEL_R |= (1<<4 | 1<<5 | 1<<6 | 1<<7 );

  // GPIO Portcontrol
  INT32U reg = GPIO_PORTB_PCTL_R;
  reg &= 0x00FFFF00;                             // Clear bits
  GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_SSI2CLK
      | GPIO_PCTL_PB5_SSI2FSS
      | GPIO_PCTL_PB6_SSI2RX
      | GPIO_PCTL_PB7_SSI2TX;      // Set bits

      // SSI 0 master mode and disable SSI
  SSI2_CR1_R = 0;

  // Configure the SSI clock source (SSICC)
  SSI2_CC_R = 0;                  // Use system clock

  // Configure the clock prescale divisor (SSICPSR)
  // SSInClk = SysClk / (CPSDVSR * (1 + SCR))
  // SSInClk = 10Mhz
  SSI2_CPSR_R = 10;       // CPSDVSR
  // SCR is deafult 0

  // Write the SSICR0 register with
  // Serial clock rate (SCR)
  // Desired clock phase/polarity, if using Freescale SPI mode (SPH and SPO)
  // The protocol mode: Freescale SPI
  // The data size (DSS)

  bit_clear (SSI2_CR0_R, SSI_CR0_SPH);    // SPH = 0
  bit_set (SSI2_CR0_R, SSI_CR0_SPO);      // SPO = 0
  bit_clear (SSI2_CR0_R, 0b110000 );      // Freescale SPI Frame Format
  bit_set (SSI2_CR0_R, SSI_CR0_DSS_16);   // DDS = 16 bit

  // Enable SSI
  bit_set (SSI2_CR1_R, SSI_CR1_SSE);
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

void init_debug_pins()
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialize the debug pins PB0, PB1 and PB3
 ******************************************************************************/
{
  // Activate PORTB Clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

  // GPIO Direction (GPIOB)
  GPIO_PORTB_DIR_R |= ( 1<<0 | 1<<1 | 1<<3 );

  // GPIO Digital Enable (GPIODEN)
  GPIO_PORTB_DEN_R |= ( 1<<0 | 1<<1 | 1<<3 ) ;
}

void sample_int_clear()
{
  TIMER3_ICR_R |= TIMER_ICR_CAECINT;
}

void sample_put(sample_t *sample)
{
  // mono operation
  INT32U mono = (sample->left + sample->right)>>2;
  TIMER3_TAMATCHR_R = (INT32U)(adc_pwm_ratio * mono);;
}

void sample_get(sample_t *sample)
{
  if( !( ADC0_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY) )
     sample->left = (INT16U)(ADC0_SSFIFO3_R);
   else
     sample->left = 2048;

   if( !( ADC1_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY) )
     sample->right = (INT16U)(ADC1_SSFIFO3_R);
   else
     sample->right = 2048;

   ADC0_PSSI_R |= ADC_PSSI_SS3;
   ADC1_PSSI_R |= ADC_PSSI_SS3;
}

void hardware_init(INT32U sample_freq)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  INT16U cycles = CPU_F / sample_freq;
  adc_pwm_ratio = (FP32)cycles/4096;

  disable_global_int();

  set_80Mhz_clock();

  // Initialize the Tiva board
  init_tiva_board();

  delay_init();

  init_PWM( cycles );

  init_debug_pins();

  init_ADC();

  enable_FPU();

  spi_init();

  init_digiswitch();

  systick_init();

  // Enable global interrupt
  enable_global_int();
}

/****************************** End Of Module *******************************/
