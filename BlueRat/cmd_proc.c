/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/17 10:00p $
 * @brief    Uart driver demo sample.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "buffer.h"
#include "uart_app.h"
#include "parser.h"
#include "timer_app.h"
#include "acmp.h"
#include "fmc.h"
#include "cmd_proc.h"

#define ISP_PASSWORD  (0x46574154)     // input password is FWAT for entering ISP  

uint8_t compare_result;

void EnterISP(void)
{
        //uint32_t   au32Config[4] = {0xffffff7f,0xffffffff,0xfffffff,0xffffffff};
        uint32_t sleep_time = 1000;

        Set_IR_Repeat_Cnt(0);
        printf("\nISP\n" );    
        while(sleep_time-->0){WDT_ResetCounter();}     // Delay for message
        Init_Parser();
        Init_Timer_App();
        Init_IR_buffer();
    
        //printf(  "\nEntering Software update mode.\n" );
        //printf(  "Please close Autobox Application and then connect to Software update tool.\n\n");
        //while(sleep_time-->0){WDT_ResetCounter();}     // Delay for message
        WDT_Close();
        SYS_UnlockReg();
       	FMC_Open();
//        FMC_EnableConfigUpdate();
//        FMC_WriteConfig(au32Config,4);
//        FMC_DisableConfigUpdate();
        FMC_SetBootSource(1);     
        //SYS_LockReg();
        // Restart system after ISP jumper removed
        //SYS_UnlockReg();
        SYS_ResetCPU();    
        SYS_LockReg();
}

void CheckIfISP(void)
{
    // If booting from LDROM, no need to check ISP pin
    //if(FMC_GetBootSource()==1)
    //{
    //    return;     
    //}

    // PB0 is low --> Enter ISP
    if(((PB->PIN)&BIT0)==0) 
    {
        EnterISP(); // system will enter LDROM afterward
    }
}

void ProcessInputCommand(void)
{
    switch(Next_Command_Get())
    {
        // STOP ALL
        case ENUM_CMD_STOP_ALL:
            //uart_output_enqueue_with_newline('Z');
            Set_IR_Repeat_Cnt(0);
            while(Get_IR_Tx_running_status()) {}        // Wait until previous Tx Finish
            Init_Parser();
            Init_Timer_App();
            Init_IR_buffer();
            break;

        case ENUM_CMD_DO_NOTHING:
            uart_output_enqueue('H');    
            uart_output_enqueue_with_newline('I');
            break;
         
        // Readback sensor value
        case ENUM_CMD_GET_SENSOR_VALUE:    
            if(compare_result)
            {
                 uart_output_enqueue_with_newline('1');
            }
            else
            {
                 uart_output_enqueue_with_newline('0');
            }
            break;

        // Read back GPIO port input value
        case ENUM_CMD_GET_GPIO_INPUT:        // Read Input Port
            {
                    // PB7/PB1/PA15/PA14/PA11/PA10
                    uint32_t input_data, temp_pa, temp_pb;
                    temp_pa = PA->PIN;
                    temp_pb = PB->PIN;
                    if(temp_pb & 0x2)               // PB1
                    { 
                        input_data = 0x10;
                    } 
                    else 
                    { 
                        input_data = 0x0; 
                    } 
                    if(temp_pb&0x80)                // PB7
                    {
                        input_data |= 0x20;
                    }
                    temp_pa &= 0xcc00; // keep PA15/PA14/PA11/PA10
                    input_data |= ((temp_pa>>10)&0x03) | ((temp_pa>>12)&0x0c);
                    uart_output_enqueue('0');
                    uart_output_enqueue('x');
                    OutputHexValue(input_data);
                    uart_output_enqueue('\n');
            }
            break;
            
        case ENUM_CMD_SET_GPIO_ALL_BIT:                 
            {
                        uint32_t output_data;
                        output_data = Next_Input_Parameter_Get()&0xff;
                        PA->DATMSK = ~(0xffUL);
                        PA->DOUT = output_data;
                        //uart_output_enqueue('P');
                        //OutputHexValue(output_data);
                        //uart_output_enqueue('\n');
            }
            break;
            
        case ENUM_CMD_SET_GPIO_SINGLE_BIT:
            {
                uint32_t output_data, bit_no, return_data;
                output_data = Next_Input_Parameter_Get() & 0xffff;
                bit_no = (output_data>>8)&0xff;
                output_data &= 0x01;
                return_data = (bit_no<<4) | output_data;
                PA->DATMSK = ~(1UL<<bit_no);
                output_data <<= bit_no;
                PA->DOUT = output_data;
                //uart_output_enqueue('p');
                //OutputHexValue(return_data);
                //uart_output_enqueue('\n');
            }
            break;

        // Add more Repeat Count
        case ENUM_CMD_ADD_REPEAT_COUNT:
            {
                uint32_t output_data = Next_Input_Parameter_Get();
                if(output_data>0)
                {
                    uint64_t temp_cnt = Get_IR_Repeat_Cnt();
                    temp_cnt += output_data;
                    if(temp_cnt>0xffffffff)
                    {
                        temp_cnt=0xffffffff;
                    }
                    Set_IR_Repeat_Cnt((uint32_t)temp_cnt);
                }
                //uart_output_enqueue('R');
                //OutputHexValue(output_data);
                //uart_output_enqueue('\n');
            }
            break;
            
        case ENUM_CMD_ENTER_ISP_MODE:
            {
                uint32_t output_data = Next_Input_Parameter_Get();
                
                if(output_data==ISP_PASSWORD)     
                {
                    EnterISP();
                }
                else
                {
                    //uart_output_enqueue_with_newline('U');
                    //OutputHexValue(Next_Command_Get());
                    //uart_output_enqueue('\n');
                }
            }
            break;
            

        case ENUM_CMD_INPUT_TX_SIGNAL:
            
            //uart_output_enqueue_with_newline('T');
            Set_IR_Repeat_Cnt(0);
            while(Get_IR_Tx_running_status()) {}        // Wait until previous Tx Finish
            Set_IR_Repeat_Cnt(Next_Repeat_Count_Get());
            Set_PWM_period(Next_PWM_Period_Get());
            Set_PWM_duty_cycle(Next_DutyCycle_Period_Get());
            Copy_Input_Data_to_Tx_Data_and_Start();
            break;

        default:
            //uart_output_enqueue_with_newline('U');
            //OutputHexValue(Next_Command_Get());
            //uart_output_enqueue('\n');
            break;
    }
}

