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

//#define     WIDTH_TIMER     (TIMER0)

uint8_t	    IR_Transmitter_Running;
uint8_t	    IR_Finish_Tx_one_RC;
uint8_t	    IR_Finish_Tx_ALL_RC;
uint32_t    IR_Repeat_Cnt;
uint32_t	PWM_period;
uint32_t	PWM_duty_cycle;
uint8_t     bLevel;

uint32_t    PWM_internal_PWM_pulse_repeat_counter;
uint32_t    PWM_internal_PWM_high_cnt;
uint32_t    PWM_internal_PWM_low_cnt;

uint32_t    Get_IR_Repeat_Cnt(void) { return IR_Repeat_Cnt; }
void        Set_IR_Repeat_Cnt(uint32_t cnt) { IR_Repeat_Cnt = cnt; }
uint32_t    Get_PWM_period(void) { return PWM_period; }
void        Set_PWM_period(uint32_t period) { PWM_period = period; }
uint32_t    Get_PWM_duty_cycle(void) { return PWM_duty_cycle; }
void        Set_PWM_duty_cycle(uint32_t duty_cycle) { PWM_duty_cycle = duty_cycle; }
//uint32_t    Get_Tx_Level(void) { return bLevel; }
//void        Set_Tx_Level(uint32_t level) { bLevel = level; }

uint32_t    SW_Timeout_Time_PB1, SW_Timeout_Time_PB7;
uint8_t     SW_Timeout_PB1, SW_Timeout_PB7;
uint32_t    SW_Timer_PB1, SW_Timer_PB7;

const uint32_t PWM_CLOCK_UNIT_DIVIDER = 8;         // pwm-clock is 1/PWM_CLOCK_UNIT_DIVIDER

void PWM_Clear_Pulse_Tuple(void)
{
    PWM_internal_PWM_pulse_repeat_counter = 0; 
    PWM_internal_PWM_high_cnt = 0;
    PWM_internal_PWM_low_cnt = 0;
}

void PWM_Set_Pulse_Tuple(T_PWM_BUFFER pwm_tuple)
{
    PWM_internal_PWM_pulse_repeat_counter = pwm_tuple.repeat_no;
    PWM_internal_PWM_high_cnt = pwm_tuple.high_cnt;
    PWM_internal_PWM_low_cnt = pwm_tuple.low_cnt;
}

void PWM_Load_Internal_Counter_and_Decrease_Internal_Count(void)
{
    PWM_SetOutputPulse_v3(PWM_internal_PWM_high_cnt,PWM_internal_PWM_low_cnt);
    --PWM_internal_PWM_pulse_repeat_counter;
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
        // Refresh PWM counter value and --internal count
        PWM_Load_Internal_Counter_and_Decrease_Internal_Count();
        
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
        // If not the last repeat time of current PWM tuple, PWM_Load_Internal_Counter_and_Decrease_Internal_Count() is sufficient 
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
    PWM_Clear_Pulse_Tuple();
    Init_TIMER1();
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
/*
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
*/
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
        PWM_Load_Internal_Counter_and_Decrease_Internal_Count();        // Load 1st PWM count

        PWM_EnableInt(PWM0, 0x0, 0);
        PWM_EnableOutput(PWM0, 0x3);
        //PWM_EnableOutput(PWM0, 0x3); // Enable Output of both Channels at once
        
        PWM_Start_v3(PWM0);
        //TIMER_Start(WIDTH_TIMER);
        PWM_Load_Internal_Counter_and_Decrease_Internal_Count();        // Pre-load 2nd PWM count, others are pre-loaded in interrupt 
	}
	else
	{
        //uart_output_enqueue_with_newline('-');       // empty Tx buffer - shouldn't be here
	}
}

//
// Timer1 is used for debouncing.
//

// GPIO part of code which is used for debouncing  
void EINT1_IRQHandler(void)     // GPIO interrupt for PB1
{
    if(GPIO_GET_INT_FLAG(PB,0x1<<1))
    {    
        SW_Timer_PB1 = SW_Timeout_Time_PB1;
        SW_Timeout_PB1 = 0;
        GPIO_CLR_INT_FLAG(PB,1<<1);     // PB1
    }    
}
 
void GPAB_IRQHandler(void)      // GPIO interrupt except PB0/PB1
{
    if(GPIO_GET_INT_FLAG(PB,0x1<<7))
    {
        SW_Timer_PB7 = SW_Timeout_Time_PB7;
        SW_Timeout_PB7 = 0;
        GPIO_CLR_INT_FLAG(PB,1<<7);     // PB7
    }   
}

// End of GPIO part of code

// Please note that we select 16KHz clock as source of Timer 1
#define TMR1_PRESCALER      (1)     // minus 1 before writing to register
#define TMR1_TICK_CNT       (2)
#define TMR1_MS_CNT_VALUE   (16/(TMR1_TICK_CNT*TMR1_PRESCALER))   // current TMR1 software timer is 1/TMR1_MS_CNT_VALUE == 1/8 ms for every tick

void Init_TIMER1(void)
{
    TIMER1->CTL = TMR_CTL_RSTCNT_Msk;
    SW_Timeout_Time_PB1 = SW_Timeout_Time_PB7 = (TIMER1_DEFAULT_TIMEOUT_TIME*TMR1_MS_CNT_VALUE);
    SW_Timer_PB1 = SW_Timer_PB7 = 0;   
    SW_Timeout_PB1 = SW_Timeout_PB7 = 1;
    TIMER1->CMP = TMR1_TICK_CNT;           // interrupt every TMR1_TICK_CNT*TMR1_PRESCALER/16 ms = 1/8 ms
    TIMER1->CTL = (TIMER_PERIODIC_MODE) | (TMR1_PRESCALER-1) | TMR_CTL_CNTEN_Msk | TMR_CTL_INTEN_Msk;
}    

uint8_t Check_PB1_Timeout_Status(void)
{
    uint8_t bRet;
    if(SW_Timeout_Time_PB1==0)  // if timeout_time is 0, always return timeout_true
    {
        bRet = 1;
    }
    else
    {
        bRet = SW_Timeout_PB1;
    }    
    return bRet;
}

uint8_t Check_PB7_Timeout_Status(void)
{
    uint8_t bRet;
    if(SW_Timeout_Time_PB7==0)  //  if timeout_time is 0, always return timeout_true
    {
        bRet = 1;
    }
    else
    {
        bRet = SW_Timeout_PB7;
    }    
    return bRet;
}

void Set_TimeoutTime_PB1(uint32_t target_timer_value) // target_timer_value unit is ms
{    
    if(target_timer_value==0)
    {
        SW_Timer_PB1 = 0;      // force to stop sw timer if timeout time is 0ms
        SW_Timeout_PB1 = 1;
        SW_Timeout_Time_PB1 = 0;
    }
    else 
    {
        if(target_timer_value>=(1<<24))
        {
            target_timer_value = (1<<24)-1;
        }
        SW_Timeout_Time_PB1 = target_timer_value;
    }
}    

void Set_TimeoutTime_PB7(uint32_t target_timer_value) // target_timer_value unit is ms
{    
    if(target_timer_value==0)
    {
        SW_Timer_PB7 = 0;      // force to stop sw timer if timeout time is 0ms
        SW_Timeout_PB7 = 1;
        SW_Timeout_Time_PB7 = 0;
    }
    else 
    {
        if(target_timer_value>=(1<<24))
        {
            target_timer_value = (1<<24)-1;
        }
        SW_Timeout_Time_PB1 = target_timer_value;
    }   
}    

void TMR1_IRQHandler(void)
{
    // Assumed that SW_Timeout_Time_PBx is non-zero,
    // When it is set to zero, SW_Timer_PBx is set to 0 & SW_Timeout_PBx to 1 
    // Therefore, we don't check if SW_Timeout_Time_PBx is zero or not
    if(SW_Timer_PB1)
    {
        SW_Timer_PB1--;
    }
    else
    {
        SW_Timeout_PB1 = 1;
    }
    
    if(SW_Timer_PB7)
    {
        SW_Timer_PB7--;
    }
    else
    {
        SW_Timeout_PB7 = 1;
    }
    TIMER_ClearIntFlag(TIMER1);	
}

///******************************************************************************
//**                            End Of File
//******************************************************************************/
