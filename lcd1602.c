/*****************************************************************************
* University of Southern Denmark
* Embedded C Programming (ECP)
*
* MODULENAME.: lcd1602.c
*
* PROJECT....: F18.GR3.Code
*
* DESCRIPTION: See module specification file (.h-file).
*
* Change Log:
******************************************************************************
* Date    Id    Change
* YYMMDD
* --------------------
* 180419  JJA   Module created.
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <lcd1602.h>
#include <stdint.h>
#include "global.h"
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "hardware.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"


/*****************************    Defines    *******************************/
#define     LCD_DATA_PORT           GPIO_PORTC_DATA_R
#define     LCD_DATA_DDR            GPIO_PORTC_DIR_R
#define     LCD_DATA_EN             GPIO_PORTC_DEN_R
#define     LCD_D4_PIN              4
#define     LCD_D5_PIN              5
#define     LCD_D6_PIN              6
#define     LCD_D7_PIN              7

#define     LCD_CTRL_PORT           GPIO_PORTD_DATA_R
#define     LCD_CTRL_DDR            GPIO_PORTD_DIR_R
#define     LCD_CTRL_EN             GPIO_PORTD_DEN_R
#define     LCD_RS_PIN              2
#define     LCD_E_PIN               3

/*****************************   Constants   *******************************/
typedef enum {HIGH, LOW} pin_state;

/*****************************   Variables   *******************************/
// Internal display buffer array
INT8U  lcd_display_buffer[LCD_BUF_SIZE];
INT32U lcd_buffer_dirty;

// Cursor position in display buffer
INT8U buffer_x = 0;
INT8U buffer_y = 0;
INT8U buffer_offset = 0;

/*****************************   Functions   *******************************/
void lcd_RS(pin_state pin)
{
  if(pin == LOW)
    bit_clear(LCD_CTRL_PORT, 1<<LCD_RS_PIN);
  else
    bit_set(LCD_CTRL_PORT, 1<<LCD_RS_PIN);
}

void lcd_E(pin_state pin)
{
  if(pin == LOW)
    bit_clear(LCD_CTRL_PORT, 1<<LCD_E_PIN);
  else
    bit_set(LCD_CTRL_PORT, 1<<LCD_E_PIN);
}

void lcd_write_nibble(INT8U byte)
{
  lcd_E(LOW);

  // clear all data bits
  LCD_DATA_PORT &= ~( 1<<LCD_D7_PIN );
  LCD_DATA_PORT &= ~( 1<<LCD_D6_PIN );
  LCD_DATA_PORT &= ~( 1<<LCD_D5_PIN );
  LCD_DATA_PORT &= ~( 1<<LCD_D4_PIN );

  // put Upper Nipple of byte on the data port
  LCD_DATA_PORT |= (byte & 1<<7 ) ? (1<<LCD_D7_PIN) : 0;
  LCD_DATA_PORT |= (byte & 1<<6 ) ? (1<<LCD_D6_PIN) : 0;
  LCD_DATA_PORT |= (byte & 1<<5 ) ? (1<<LCD_D5_PIN) : 0;
  LCD_DATA_PORT |= (byte & 1<<4 ) ? (1<<LCD_D4_PIN) : 0;
  delay_us(1);

  // Set E low
  lcd_E(HIGH);
  delay_us(1);
  // Set E high
  lcd_E(LOW);
  delay_us(1);
}

void lcd_direct_write_instruction(INT8U byte)
{
  lcd_RS(LOW);
  lcd_E(LOW);
  lcd_write_nibble(byte);
  lcd_write_nibble(byte<<4);
  delay_us(41);
}

void lcd_direct_write_data(INT8U byte)
{
  lcd_RS(HIGH);
  lcd_E(LOW);
  lcd_write_nibble(byte);
  lcd_write_nibble(byte<<4);
  delay_us(41);
}

void lcd_direct_write_data_nodelay(INT8U byte)
/*****************************************************************************
 *   Input    : Byte containing the data
 *   Output   : -
 *   Function : Write data directly to LCD without end delay
 ******************************************************************************/
{
  lcd_RS(HIGH);
  lcd_E(LOW);
  lcd_write_nibble(byte);
  lcd_write_nibble(byte<<4);
}

void lcd_direct_set_cursor(INT8U x, INT8U y)
{
  INT8U ddram_adress = 0x80;
  ddram_adress |= (y*0x40 + x);
  lcd_direct_write_instruction(ddram_adress);
}

void lcd_direct_write_line(char *line)
{
  INT8U i = 0;
  while( line[i] != 0)
  {
    lcd_direct_write_data(line[i++]);
  }
}

void lcd_direct_clear(void)
{
  // Clear entire display
  lcd_direct_write_instruction(0x01);
  delay_us(1520);
}

void lcd_set_cursor(INT8U x, INT8U y)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  if(  y < LCD_LINES && x <= LCD_CHARS )
  {
    buffer_x = x;
    buffer_y = y;
    buffer_offset = (y<<4)+x;
  }
}

void lcd_clear()
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  for(INT8U i = 0; i < LCD_BUF_SIZE; i++ )
    lcd_display_buffer[i] = 0x20;         // Clear with space
  buffer_x = 0;
  buffer_y = 0;
  buffer_offset = 0;
  lcd_buffer_dirty = 0;
}

void lcd_write(char *str)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  if( sizeof(*str) < (LCD_BUF_SIZE - buffer_offset ) )
  {
    INT8U len = strlen(str);
    memcpy( &lcd_display_buffer[buffer_offset] , str, len);
    buffer_offset += len;
  }
}

void lcd_write_char(char ch)
/*****************************************************************************
 *   Header description
 ******************************************************************************/
{
  lcd_display_buffer[buffer_offset] = ch;
  buffer_offset = buffer_offset < LCD_BUF_SIZE ? buffer_offset+1 : 0;
}

void lcd_buffer_task(void *pvParameters)
{
  const TickType_t xDelay = 2 / portTICK_PERIOD_MS;

  while(1)
  {
    static INT8U state = 0;

    if(state < LCD_BUF_SIZE)
    {
      if(state == 0)
        lcd_direct_set_cursor(0,0);
      if(state == 16)
        lcd_direct_set_cursor(0,1);

      lcd_direct_write_data_nodelay( lcd_display_buffer[state] );
      state++;
    }
    else
      state = 0;

    vTaskDelay( xDelay );

  }
}

void lcd_set_custom_font()
{
  lcd_direct_write_instruction(0x40);
  lcd_direct_write_data(0b10000);
  lcd_direct_write_data(0b11000);
  lcd_direct_write_data(0b11100);
  lcd_direct_write_data(0b11110);
  lcd_direct_write_data(0b11100);
  lcd_direct_write_data(0b11000);
  lcd_direct_write_data(0b10000);
  lcd_direct_write_data(0b00000);


  //lcd_write_instruction(0x41);
  lcd_direct_write_data(0b00001);
  lcd_direct_write_data(0b00011);
  lcd_direct_write_data(0b00111);
  lcd_direct_write_data(0b01111);
  lcd_direct_write_data(0b00111);
  lcd_direct_write_data(0b00011);
  lcd_direct_write_data(0b00001);
  lcd_direct_write_data(0b00000);
}

void lcd_direct_write_buffer(INT8U *buffer)
{
  for(INT8U y=0; y < LCD_LINES; y++)
  {
    lcd_direct_set_cursor(0, y);
    for(INT8U x=0 ; x < LCD_CHARS ; x++)
    {
      lcd_direct_write_data( buffer[x+(16*y)] );
    }
  }
}

BaseType_t lcd_init()
{
  BaseType_t return_value = pdTRUE;

  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3;

  // Init GPIO pins for LCD display
  LCD_DATA_DDR |= (1<<LCD_D4_PIN) | (1<<LCD_D5_PIN) | (1<<LCD_D6_PIN) |
      (1<<LCD_D7_PIN);
  LCD_DATA_EN  |= (1<<LCD_D4_PIN) | (1<<LCD_D5_PIN) | (1<<LCD_D6_PIN) |
      (1<<LCD_D7_PIN);

  LCD_CTRL_DDR |= (1<<LCD_E_PIN) | (1<<LCD_RS_PIN);
  LCD_CTRL_EN  |= (1<<LCD_E_PIN) | (1<<LCD_RS_PIN);

  // Begin LCD INIT
  // wait 40ms
  delay_us(40000);
  lcd_RS(LOW);
  lcd_E(LOW);

  // Write function set
  lcd_write_nibble(0x30);

  // Wait 4.1ms
  delay_us(4100);

  // Write function set
  lcd_write_nibble(0x30);

  // Wait 100us
  delay_us(100);

  // Write function set
  lcd_write_nibble(0x30);

  // Wait 100us
  delay_us(100);

  // Write 4bit function set
  lcd_write_nibble(0x20);
  delay_us(37);

  // Functions set 0 0 1 DL N F - -
  // DL : 0: 4 bit     1: 8 bit       Datalines
  // N  : 0: 1 line    1: 2 lines     Display lines
  // F  : 0: 5x8       1: 5x10        Fontsize
  lcd_direct_write_instruction(0b00101000);

  // Display  0 0 0 0 1 D C B
  // D  :                             Display on/off
  // C  :                             Cursor on/off
  // B  :                             Blinking on/off
  lcd_direct_write_instruction(0b00001000);


  // Clear entire display
  lcd_direct_write_instruction(0x01);
  delay_us(1520);

  // Entry mode set 0 0 0 0 0 1 I/D S
  // I/D  0: Dec          1: Inc
  // S    0: Noshift
  lcd_direct_write_instruction(0b00000110);

  // LCD ON
  lcd_direct_write_instruction(0b00001100);

  // set cursor to 0,0
  lcd_direct_set_cursor(0,0);

  lcd_clear();

  lcd_set_custom_font();

  return_value &= xTaskCreate( lcd_buffer_task , "LCD",
                                 USERTASK_STACK_SIZE, NULL, MED_PRIO, NULL);

  return return_value;
}
/****************************** End Of Module *******************************/












