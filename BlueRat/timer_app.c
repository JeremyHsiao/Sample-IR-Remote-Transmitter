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
uint8_t	    IR_Finish_Tx_ALL_RC;
uint32_t    IR_Repeat_Cnt;
uint32_t	PWM_period;
uint32_t	PWM_duty_cycle;
uint8_t     bLevel;

uint32_t    PWM_internal_PWM_pulse_repeat_counter = 0;
uint32_t    PWM_internal_PWM_high_cnt = 0;
uint32_t    PWM_internal_PWM_low_cnt = 0;

uint32_t    Get_IR_Repeat_Cnt(void) { return IR_Repeat_Cnt; }
void        Set_IR_Repeat_Cnt(uint32_t cnt) { IR_Repeat_Cnt = cnt; }
uint32_t    Get_PWM_period(void) { return PWM_period; }
void        Set_PWM_period(uint32_t period) { PWM_period = period; }
uint32_t    Get_PWM_duty_cycle(void) { return PWM_duty_cycle; }
void        Set_PWM_duty_cycle(uint32_t duty_cycle) { PWM_duty_cycle = duty_cycle; }
//uint32_t    Get_Tx_Level(void) { return bLevel; }
//void        Set_Tx_Level(uint32_t level) { bLevel = level; }

const uint32_t PWM_CLOCK_UNIT_DIVIDER = 8;         // pwm-clock is 1/PWM_CLOCK_UNIT_DIVIDER

void PWM_Set_Pulse_Tuple(T_PWM_BUFFER pwm_tuple)
{
    PWM_internal_PWM_pulse_repeat_counter = pwm_tuple.repeat_no;
    PWM_internal_PWM_high_cnt = pwm_tuple.high_cnt;
    PWM_internal_PWM_low_cnt = pwm_tuple.low_cnt;
}

void PWM_Load_Internal_Counter(void)
{
    PWM_SetOutputPulse_v3(PWM_internal_PWM_high_cnt,PWM_internal_PWM_low_cnt);
}

void PWM0_IRQHandler(void)
{
    if((PWM_internal_PWM_pulse_repeat_counter==0)&&(IR_Repeat_Cnt==0))         
    {
        // End of all data, stop PWM
        PWM0->CTL &= ~(PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTEN1_Msk);  // stop-run for both PWM
        PWM_DisableInt(PWM0, 0x0);
        IR_Transmitter_Running = 0;
    }
    else
    {
        // Refresh PWM counter value
        PWM_Load_Internal_Counter();
        --PWM_internal_PWM_pulse_repeat_counter;
        if(PWM_internal_PWM_pulse_repeat_counter==0)         
        {
            // If this time is the last repeat time of current PWM tuple,  load next tuple
            T_PWM_BUFFER temp_pwm;
            if(PWM_Pulse_read(&temp_pwm)!=FALSE)   
            {
                // not end of pulse, load next tuple
                PWM_Set_Pulse_Tuple(temp_pwm);
            }
            else
            {   
                // end of pulse, check if IR_Repeat_Cnt is > zero
                if(IR_Repeat_Cnt>0)
                {
                    IR_Repeat_Cnt--;
                    // if yes, reset IR data pointer and start to load again
                    PWM_Pulse_restart_read_pointer();
                    PWM_Pulse_read(&temp_pwm);
                    PWM_Set_Pulse_Tuple(temp_pwm);
                }
                else
                {
                    // no more to repeat, Follow instruction of Nuvoton: set period=0 and disable PWM at next interrupt
                    PWM0->PERIOD0 = 0;
                    PWM0->PERIOD1 = 0;
                    IR_Finish_Tx_ALL_RC = 1;
                }
                IR_Finish_Tx_one_RC = 1;
            }
        }
        // If not the last repeat time of current PWM tuple, PWM_Load_Internal_Counter() is sufficient 
    }
    PWM_ClearIntFlag(PWM0, 0);
    WDT_ResetCounter();
}

void Init_Timer_App(void)
{
    IR_Transmitter_Running = 0;
    IR_Finish_Tx_one_RC = 0;
    IR_Finish_Tx_ALL_RC = 0;
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

void Clear_IR_Tx_All_Finish(void)
{
    IR_Finish_Tx_ALL_RC = 0;
}

uint8_t Get_IR_Tx_Finish_All_status(void)
{
    return IR_Finish_Tx_ALL_RC;
}

void Restart_IR_Pulse(void)
{
    PWM_SetOutputPulse_v2(PWM0, 20, 0);
    bLevel = 1;
}

void Setup_IR_PWM_Pulse(void)
{
	if(bLevel)
	{
        if(PWM_period!=0)
        {
            PWM_SetOutputPulse_v2(PWM0, PWM_period, Get_PWM_duty_cycle());
        }
        else
        {
            PWM_SetOutputPulse_v2(PWM0, 20, 100);
        }
        //PB->DOUT |= 0x08UL;
	}
	else
	{
        if(PWM_period!=0)
        {
            PWM_SetOutputPulse_v2(PWM0, PWM_period, 0);
        }
        else
        {
            PWM_SetOutputPulse_v2(PWM0, 20, 0);
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
        IR_output_read(&temp_width);
        Timer_SetNextTimeout(WIDTH_TIMER,temp_width);
		Setup_IR_PWM_Pulse();
    }
    else
    {
        if(IR_Repeat_Cnt>0)
        {
            IR_Repeat_Cnt--;
            IR_output_restart_read_pointer();
            IR_output_read(&temp_width);
            Timer_SetNextTimeout(WIDTH_TIMER,temp_width);
            Setup_IR_PWM_Pulse();
            WDT_ResetCounter();
       }
        else
        {
            // No more data
            IR_Transmitter_Running = 0;
            PWM_Stop_v2(PWM0);
            TIMER_Stop(WIDTH_TIMER);
            TIMER_DisableInt(WIDTH_TIMER);
            Restart_IR_Pulse();
            IR_Finish_Tx_ALL_RC = 1;
        }
        IR_Finish_Tx_one_RC = 1;
    }
    TIMER_ClearIntFlag(TIMER0);	
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
        IR_output_read(&temp_width);
        Timer_Init();    
        Timer_SetNextTimeout(WIDTH_TIMER,temp_width);
        Restart_IR_Pulse();
        TIMER_EnableInt(WIDTH_TIMER);
        TIMER_Start(WIDTH_TIMER);
        Setup_IR_PWM_Pulse();
        PWM_Start_v2(PWM0);
        IR_Transmitter_Running = 1;
	}
	else
	{
        //uart_output_enqueue_with_newline('-');       // empty Tx buffer
	}
}

void PWM_Transmit_Buffer_StartSend(void)
{
    //uint32_t temp_width;
    T_PWM_BUFFER temp_pwm;
	if (IR_Transmitter_Running!=0)
	{
	  return;			// already running, no need to start timer match and interrupt
	}

    PWM_Pulse_restart_read_pointer();
	// Get next pulse width and level
	if(PWM_Pulse_read(&temp_pwm)!=FALSE) 
	{
		// Load 1st PWM value tuple
        IR_Transmitter_Running = 1;
        
        PWM_ConfigOutputChannel_v3(PWM0); 
        
        //PWM_Pulse_read(&temp_pwm);
        PWM_Set_Pulse_Tuple(temp_pwm);
        PWM_Load_Internal_Counter();        // Load 1st PWM count, others are in interrupt 

        PWM_EnableInt(PWM0, 0x0, 0);
        PWM_EnableOutput(PWM0, 0x3);
        //PWM_EnableOutput(PWM0, 0x3); // Enable Output of both Channels at once
        
        PWM_Start_v3(PWM0);
        //TIMER_Start(WIDTH_TIMER);
        
	}
	else
	{
        //uart_output_enqueue_with_newline('-');       // empty Tx buffer - shouldn't be here
	}
}

///******************************************************************************
//**                            End Of File
//******************************************************************************/
