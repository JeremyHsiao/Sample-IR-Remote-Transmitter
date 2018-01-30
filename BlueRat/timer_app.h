/****************************************************************************
 *   $Id:: timer_app.h 3635 2010-06-02 00:31:46Z usb00423                     $
 *   Project: IR Tx
 *
 *   Description:
 *     This file contains definition and prototype for 32-bit timer 
 *     configuration.
 *
 ****************************************************************************/
#ifndef _TIMER_APP_H_
#define _TIMER_APP_H_

extern void         Init_Timer_App(void);
extern void 		IR_Transmit_Buffer_StartSend(void);
extern void         PWM_Transmit_Buffer_StartSend(void);
extern uint8_t      Get_IR_Tx_running_status(void);
extern void         Reset_IR_Tx_running_status(void);

extern void         Clear_IR_Tx_Finish(void);
extern uint8_t      Get_IR_Tx_Finish_status(void);

extern void         Clear_IR_Tx_All_Finish(void);
extern uint8_t      Get_IR_Tx_Finish_All_status(void);

extern uint32_t     Get_IR_Repeat_Cnt(void);
extern void         Set_IR_Repeat_Cnt(uint32_t cnt);
extern uint32_t     Get_PWM_period(void);
extern void         Set_PWM_period(uint32_t period);
extern uint32_t     Get_PWM_duty_cycle(void);
extern void         Set_PWM_duty_cycle(uint32_t duty_cycle);

extern void         Setup_IR_PWM_Pulse(void);
extern void         Timer_Init(void);

extern const uint32_t PWM_CLOCK_UNIT_DIVIDER; 

#endif /* end _TIMER_APP_H_ */
/*****************************************************************************
**                            End Of File
******************************************************************************/
