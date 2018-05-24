/*****************************************************************************
 * University of Southern Denmark
 *
 * MODULENAME.: ui.c
 *
 * PROJECT....: F18.GR3
 *
 * DESCRIPTION: 
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * --------------------
 * May 9, 2018  jorn    Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "lcd1602.h"
#include "hid.h"
#include "hardware.h"
#include "systick.h"
#include "audio.h"
#include "mod_vol.h"
#include "mod_echo.h"
#include "mod_reverb.h"
#include "mod_filter.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
typedef struct menuitem
{
  char name[15];
  BOOLEAN (*function)(void);
  BOOLEAN (*sw2_func)(void);
  struct menu *childmenu;
  struct menuitem *next;
  struct menuitem *prev;
} menuitem_t;

typedef struct menu
{
  struct menu *parentmenu;
  struct menuitem *menuitem;
} menu_t;

menu_t *rootmenu = NULL;
BOOLEAN visible = TRUE;

enum views
{
  MENU, STATUS, FUNCTION, SW2_FUNC
} view = MENU;

/*****************************   Functions   *******************************/

menuitem_t* create_menuitem(char *name)
{
  menuitem_t *menuitem = (menuitem_t*) malloc(sizeof(menuitem_t));
  strcpy(menuitem->name, name);
  menuitem->childmenu = NULL;
  menuitem->function = NULL;
  menuitem->sw2_func = NULL;
  menuitem->next = NULL;
  menuitem->prev = NULL;
  return (menuitem);
}

menu_t* create_menu()
{
  menu_t *menu = (menu_t*) malloc(sizeof(menu_t));
  menu->parentmenu = NULL;
  menu->menuitem = NULL;
  return (menu);
}

menuitem_t* find_last_menitem(menu_t *menu)
{
  menuitem_t *menuitem = NULL;

  if (menu->menuitem)
  {
    menuitem = menu->menuitem;
    while (menuitem->next)
    {
      menuitem = menuitem->next;
    }
  }

  return (menuitem);
}

void add_menuitem(menu_t *menu, menuitem_t *menuitem)
{
  if (!menu->menuitem)
  {
    menu->menuitem = menuitem;
    menuitem->prev = NULL;
  }
  else
  {
    menuitem_t *last_menuitem = find_last_menitem(menu);
    last_menuitem->next = menuitem;
    menuitem->prev = last_menuitem;
    menuitem->next = NULL;
  }
}

void ui_init(void)
{
  rootmenu = create_menu();
  menuitem_t *menuitem = create_menuitem("Master Volume");
  menuitem->function = mod_vol_setvol;
  add_menuitem(rootmenu, menuitem);

  menuitem = create_menuitem("Echo");
  add_menuitem(rootmenu, menuitem);

  menu_t *echomenu = create_menu();
  echomenu->parentmenu = rootmenu;
  menuitem->childmenu = echomenu;

  menuitem = create_menuitem("Echo gain");
  menuitem->function = mod_echo_setgain;
  add_menuitem(echomenu, menuitem);

  menuitem = create_menuitem("Echo delay");
  menuitem->function = mod_echo_setdelay;
  add_menuitem(echomenu, menuitem);

  menuitem = create_menuitem("<-Back");
  menuitem->childmenu = rootmenu;
  add_menuitem(echomenu, menuitem);

  menuitem = create_menuitem("Reverb");
  add_menuitem(rootmenu, menuitem);

  menu_t *revbmenu = create_menu();
  echomenu->parentmenu = rootmenu;
  menuitem->childmenu = revbmenu;

  menuitem = create_menuitem("Reverb gain");
  menuitem->function = mod_reverb_setgain;
  add_menuitem(revbmenu, menuitem);

  menuitem = create_menuitem("Reverb delay");
  menuitem->function = mod_reverb_setdelay;
  add_menuitem(revbmenu, menuitem);

  menuitem = create_menuitem("Filter");
  add_menuitem(rootmenu, menuitem);

  menu_t *filtermenu = create_menu();
  filtermenu->parentmenu = rootmenu;
  menuitem->childmenu = filtermenu;

  menuitem = create_menuitem("Filter cutoff");
  menuitem->function = mod_filter_setfcutoff;
  menuitem->sw2_func = mod_filter_init;
  add_menuitem(filtermenu, menuitem);

  menuitem = create_menuitem("<-Back");
  menuitem->childmenu = rootmenu;
  add_menuitem(revbmenu, menuitem);

  menuitem = create_menuitem("System info");
  add_menuitem(rootmenu, menuitem);
}

void view_menu()
{
  visible = TRUE;
}

void hide_menu()
{
  visible = FALSE;
}

void ui_task(void *pvParameters)
{
  static menuitem_t *current_menuitem = NULL;
  static menu_t *current_menu = NULL;
  struct hid_msg msg;
  static uint8_t blink;

  ui_init();

  if (rootmenu)
  {
    current_menu = rootmenu;
    current_menuitem = current_menu->menuitem;
  }

  while (1)
  {
    blink = ((ticks() % 1000) > 250);
    switch (view)
    {
      case FUNCTION:
        if(current_menuitem->function)
          if(!current_menuitem->function())
            view = MENU;
        break;

      case SW2_FUNC:
        if(current_menuitem->sw2_func)
          if(!current_menuitem->sw2_func())
            view = MENU;
        break;

      case STATUS:
        lcd_clear();
        lcd_write("Audio %: ");
        char buffer[3];
        lcd_write(itoa(get_cpu_load(), buffer, 10));

        if (hid_get(&msg))
        {
          if (msg.function == HID_FUNC_SW1 && msg.event == HID_EVENT_CLK)
            view = MENU;
        }
        break;

      case MENU:
        if (hid_get(&msg))
        {
          switch (msg.function)
          {
            case HID_FUNC_SW1:
              if (msg.event == HID_EVENT_CLK)
                view = STATUS;
              break;
            case HID_FUNC_SW2:
              if(current_menuitem->sw2_func)
                view = SW2_FUNC;
              break;
            case HID_FUNC_DIGIRIGHT:
              if (current_menuitem->next)
                current_menuitem = current_menuitem->next;
              break;
            case HID_FUNC_DIGILEFT:
              if (current_menuitem->prev)
                current_menuitem = current_menuitem->prev;
              break;
            case HID_FUNC_DIGIP2:
              switch (msg.event)
              {
                case HID_EVENT_CLK:
                  if (current_menuitem->childmenu)
                  {
                    current_menu = current_menuitem->childmenu;
                    current_menuitem = current_menu->menuitem;
                  }
                  else if (current_menuitem->function)
                    view = FUNCTION;
                  break;
                case HID_EVENT_LONG:
                  current_menu = rootmenu;
                  current_menuitem = rootmenu->menuitem;
                  break;
              }
              break;
          }
        }
        lcd_clear();

        if (current_menuitem->next)
        {
          lcd_set_cursor(0, 0);
          lcd_write_char(blink ? 0x00 : ASCII_SPACE);
          lcd_write(current_menuitem->name);
          if (current_menuitem->next)
          {
            lcd_set_cursor(1, 1);
            lcd_write(current_menuitem->next->name);
          }
        }
        else if (current_menuitem->prev)
        {
          lcd_set_cursor(1, 0);
          lcd_write(current_menuitem->prev->name);
          lcd_set_cursor(0, 1);
          lcd_write_char(blink ? 0x00 : ASCII_SPACE);
          lcd_write(current_menuitem->name);
        }
        else
        {
          lcd_set_cursor(0, 0);
          lcd_write_char(blink ? 0x00 : ASCII_SPACE);
          lcd_write(current_menuitem->name);
        }
        break;
    }
    vTaskDelay(20);
  }
}

/****************************** End Of Module *******************************/
