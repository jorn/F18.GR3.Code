/*****************************************************************************
 * University of Southern Denmark
 * Embedded Programming (EMP)
 *
 * MODULENAME.: mod_filter.c
 *
 * PROJECT....: F18.GR3.Code
 *
 * DESCRIPTION: See module specification file (.h-file).
 *
 * Change Log:
 ******************************************************************************
 * Date			Id	  Change
 * MMM DD, YYYY
 * ----------------------------------------------------------------------------
 * 20 May 2018	FrH   Module created.
 *
 *****************************************************************************/

/***************************** Include files ********************************/
#include "mod_filter.h"
#include <math.h>
#include <stdlib.h>
/*****************************    Defines    ********************************/
#define FILTER_ORDER 40
#define F_SAMPLE     44100
/*****************************   Constants   ********************************/

/*****************************   Variables   ********************************/
fp_sample_t x_buffer[FILTER_ORDER];
float filter_coeffs[FILTER_ORDER];
uint8_t ndx = 0;
uint16_t param_fcutoff;
/*****************************   Functions   *********************************
 *   Function : See General module specification (general.h-file).
 *****************************************************************************/

void mod_filter(fp_sample_t *in, fp_sample_t *out)
{
    fp_sample_t fp_sample;
    sample_buffer_put_z(in, ndx);

    ndx++;
    if (ndx == 100)
    {
        ndx = 0;
    }

    x_buffer[0] = *in;
    fp_sample.left_fp32 = 0;
    fp_sample.right_fp32 = 0;

    for (int i = 0; i < FILTER_ORDER; ++i)
    {
        fp_sample.left_fp32 += filter_coeffs[i] * x_buffer[i].left_fp32;
        fp_sample.right_fp32 += filter_coeffs[i] * x_buffer[i].right_fp32;
    }

    for (int i = ( FILTER_ORDER - 1); i > 0; --i)
    {
        x_buffer[i] = x_buffer[i - 1];
    }

    out->left_fp32 = fp_sample.left_fp32;
    out->right_fp32 = fp_sample.right_fp32;
}

BOOLEAN mod_filter_setfcutoff(void)
{
    BOOLEAN ret = TRUE;
    struct hid_msg msg;
    char buffer[3];

    lcd_clear();
    lcd_write("Cutoff frequency (Hz)");
    lcd_set_cursor(0, 1);
    lcd_write("(1103-19854):");
    lcd_write( itoa(param_fcutoff, buffer, 10));

    if (hid_get(&msg))
    {
        if (msg.ch)
        {
            lcd_write_char(msg.ch);
        }
        if (msg.ch == ASCII_HASH)
        {
            ret = FALSE;
        }
        if (msg.function == HID_FUNC_DIGILEFT)
        {
            param_fcutoff =
                    param_fcutoff > 1103 ? (param_fcutoff - 1103) : 1103;
        }
        if (msg.function == HID_FUNC_DIGIRIGHT)
        {
            param_fcutoff =
                    param_fcutoff < 19854 ? (param_fcutoff + 1103) : 19854;
        }
        if (msg.function == HID_FUNC_DIGIP2 && msg.event == HID_EVENT_CLK)
        {
            ret = FALSE;
        }
    }

    return (ret);
}

void mod_filter_init(void)
{
    float Omega_c = 2 * M_PI * param_fcutoff / F_SAMPLE;
    float Omega_k[(FILTER_ORDER - 1) / 2];
    uint8_t M = ((FILTER_ORDER - 1) / 2);
    float tmp;

    for (uint8_t i = 0; i < M; ++i)
    {
        Omega_k[i] = 2 * M_PI * i / ((FILTER_ORDER - 1) / 2);
        if (Omega_k[i] <= Omega_c)
        {
            Omega_k[i] = 1;
        }

        else
        {
            Omega_k[i] = 0;
        }
    }

    for(int i = 0; i <= M; ++i)
    {
        if (i <= M)
        {
            for (int j = 0; j < M; ++j)
            {
                tmp += Omega_k[j] * cos( (2 * M_PI * j * (i - M) ) / FILTER_ORDER );
            }
            filter_coeffs[i] = (1.F / FILTER_ORDER) * ( Omega_k[0] + 2 * tmp );
        }

        else
        {
            filter_coeffs[i] = filter_coeffs[2 * M - i];
        }
    }

}

/****************************** End Of Module *******************************/
