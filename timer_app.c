/****************************************************************************
 *   $Id:: timer3app.c 3635 2010-06-02 00:31:46Z                      $
 *   Project: IR Tx
 *
 *   Description:
 *     This file contains 32-bit timer code example which include timer
 *     initialization, timer interrupt handler, and related APIs for
 *     timer setup.
 *
****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "buffer.h"
#include "uart_app.h"
#include "parser.h"
#include "timer_app.h"

#define     WIDTH_TIMER     (TIMER0)

uint8_t	    IR_Transmitter_Running;
uint8_t	    IR_Finish_Tx_one_RC;
uint32_t    IR_Repeat_Cnt;
uint32_t	PWM_period;
uint32_t	PWM_duty_cycle;
uint8_t     bLevel;

uint32_t    Get_IR_Repeat_Cnt(void) { return IR_Repeat_Cnt; }
void        Set_IR_Repeat_Cnt(uint32_t cnt) { IR_Repeat_Cnt = cnt; }
uint32_t    Get_PWM_period(void) { return PWM_period; }
void        Set_PWM_period(uint32_t period) { PWM_period = period; }
uint32_t    Get_PWM_duty_cycle(void) { return PWM_duty_cycle; }
void        Set_PWM_duty_cycle(uint32_t duty_cycle) { PWM_duty_cycle = duty_cycle; }
//uint32_t    Get_Tx_Level(void) { return bLevel; }
//void        Set_Tx_Level(uint32_t level) { bLevel = level; }

void Init_Timer_App(void)
{
    IR_Transmitter_Running = 0;
    IR_Finish_Tx_one_RC = 0;
    IR_Repeat_Cnt = 0;
    PWM_period =  (uint32_t) (1000000/(38000));		// For 38KHz PWM pulse
    PWM_duty_cycle = 33;
    bLevel = 1;
}

void Reset_IR_Tx_running_status(void)
{
    IR_Transmitter_Running = 0;
}

uint8_t Get_IR_Tx_running_status(void)
{
    return IR_Transmitter_Running;
}

void Clear_IR_Tx_Finish(void)
{
    IR_Finish_Tx_one_RC = 0;
}

uint8_t Get_IR_Tx_Finish_status(void)
{
    return IR_Finish_Tx_one_RC;
}

void Restart_IR_Pulse(void)
{
    PWM_SetOutputPulse(PWM0, PWM_CH0, 20, 0);
    PWM_SetOutputPulse(PWM0, PWM_CH1, 20, 0);
    bLevel = 1;
}

void Setup_IR_PWM_Pulse(void)
{
	if(bLevel)
	{
        if(PWM_period!=0xffffffff)
        {
            PWM_SetOutputPulse(PWM0, PWM_CH0, PWM_period, Get_PWM_duty_cycle());
            PWM_SetOutputPulse(PWM0, PWM_CH1, PWM_period, Get_PWM_duty_cycle());
        }
        else
        {
            PWM_SetOutputPulse(PWM0, PWM_CH0, 20, 100);
            PWM_SetOutputPulse(PWM0, PWM_CH1, 20, 100);
        }
        //PB->DOUT |= 0x08UL;
	}
	else
	{
        if(PWM_period!=0xffffffff)
        {
            PWM_SetOutputPulse(PWM0, PWM_CH0, PWM_period, 0);
            PWM_SetOutputPulse(PWM0, PWM_CH1, PWM_period, 0);
        }
        else
        {
            PWM_SetOutputPulse(PWM0, PWM_CH0, 20, 0);
            PWM_SetOutputPulse(PWM0, PWM_CH1, 20, 0);
        }
        //PB->DOUT &= ~0x08UL;
	}
    bLevel=!bLevel;
}

#define u32Prescale     (12)
#define u64TIMER_CLK    (((uint64_t)(49152000UL)/u32Prescale))

void Timer_Init()
{    
    WIDTH_TIMER->CTL = TMR_CTL_RSTCNT_Msk;
    WIDTH_TIMER->CTL = (TIMER_CONTINUOUS_MODE) | (u32Prescale-1) | TMR_CTL_CNTDATEN_Msk;
    WIDTH_TIMER->CMP = 0;
}

void Timer_SetNextTimeout(TMR_T *timer, uint32_t duration_us)
{
    timer->CMP += (uint32_t)(((u64TIMER_CLK/1000) * ((uint64_t)duration_us))/1000);
}

void TMR0_IRQHandler(void)
{
    uint32_t    temp_width;
    if(!IR_output_end_of_data())
    {
 		// set IR output pulse
		Setup_IR_PWM_Pulse();
        IR_output_read(&temp_width);
        Timer_SetNextTimeout(WIDTH_TIMER,temp_width);
    }
    else
    {
        if(IR_Repeat_Cnt>0)
        {
            IR_Repeat_Cnt--;
            IR_output_restart_read_pointer();
            Setup_IR_PWM_Pulse();
            IR_output_read(&temp_width);
            Timer_SetNextTimeout(WIDTH_TIMER,temp_width);
            WDT_ResetCounter();
       }
        else
        {
            // No more data
            IR_Transmitter_Running = 0;
            PWM_Stop(PWM0, 0x0);
            PWM_Stop(PWM0, 0x1);
            TIMER_Stop(WIDTH_TIMER);
            TIMER_DisableInt(WIDTH_TIMER);
            Restart_IR_Pulse();
        }
        IR_Finish_Tx_one_RC = 1;
    }
    TIMER_ClearIntFlag(WIDTH_TIMER);	
}


void IR_Transmit_Buffer_StartSend(void)
{
    uint32_t temp_width;
	if (IR_Transmitter_Running!=0)
	{
	  return;			// already running, no need to start timer match and interrupt
	}

    IR_output_restart_read_pointer();
	// Get next pulse width and level
	if(!IR_output_end_of_data())
	{
		// Please set up first pulse here
        Timer_Init();    
        IR_output_read(&temp_width);
        Timer_SetNextTimeout(WIDTH_TIMER,temp_width);

        Restart_IR_Pulse();
        Setup_IR_PWM_Pulse();
        IR_Transmitter_Running = 1;
        TIMER_EnableInt(WIDTH_TIMER);
        PWM_Start(PWM0, 0x0);
        PWM_Start(PWM0, 0x1);
        TIMER_Start(WIDTH_TIMER);
	}
	else
	{
        uart_output_enqueue_with_newline('-');       // empty Tx buffer
	}
}

///******************************************************************************
//**                            End Of File
//******************************************************************************/
