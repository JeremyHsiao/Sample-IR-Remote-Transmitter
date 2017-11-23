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

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKSEL0_HIRCFSEL_48M, CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0CH01SEL_HCLK, 0);

    /* Reset PWM0 channel0~channel3 */
    SYS_ResetModule(PWM0_RST);
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
	  SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	  SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

    /* Set GPA multi-function pins for PWM0 Channel0 */
      SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA12MFP_Msk) ) | SYS_GPA_MFP_PA12MFP_PWM0CH0;
	  SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk) ) | SYS_GPB_MFP_PB4MFP_PWM0CH0_INV;

/* Lock protected registers */
    SYS_LockReg();
}

/* Main */
int main(void)
{
	uint32_t    IR_Repeat_Cnt;
 	uint32_t	PWM_period =  (uint32_t) (480000/(38000));		// For 38KHz PWM pulse
 	uint32_t	PWM_duty_cycle = 50;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    UART_init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n+------------------------------+\n");
    printf(  "| Welcome to My IR Transmitter |\n");
    printf(  "+------------------------------\n");
	
    Initialize_buffer();
	IR_Repeat_Cnt = 0;

    /* set PWM0 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM0, PWM_CH0, 38000, 33); // Do not change 3rd/4th parameter because this function has been tailored to specific input parameter range
        
    /* Enable PWM Output path for PWM0 channel 0 */
    PWM_EnableOutput(PWM0, 0x1);

    // Start
    PWM_Start(PWM0, 0x1);

    while(1)
    {
        while(!uart_input_queue_empty_status())
        {
            uint8_t     input_char;
            input_char = uart_input_dequeue();
            ProcessInputChar(input_char);

            if(CheckSum_Ready()==true)
            {
                if (Read_CheckSum()==0)
                {
                    switch(Read_CMD_Status())
                    {
                        case ENUM_CMD_STOP_CMD_RECEIVED:
                            IR_Repeat_Cnt=0;
                            Clear_CMD_Status();
                            // Wait until previous Tx Finish -- to be implemented
                            Init_ProcessInputChar_State();
                            IR_output_restart_read_pointer();
                            // Debug message
                            {
                                uart_output_enqueue('S');
                                uart_output_enqueue('\n');
                            }
                            break;

                        case ENUM_CMD_REPEAT_COUNT_RECEIVED:
                            IR_Repeat_Cnt += Next_Repeat_Count_Get();
                            Next_Repeat_Count_Set(0);
                            Clear_CMD_Status();
                            // Debug message
                            {
                                uart_output_enqueue('B');
                                uart_output_enqueue('\n');
                            }
                            break;

                        case ENUM_CMD_WIDTH_DATA_READY:
                            // Wait until previous Tx Finish -- to be implemented
                            PWM_period = Next_PWM_Period_Get();
                            PWM_duty_cycle = Next_DutyCycle_Period_Get();
                            //IR_Transmit_Buffer_StartSend();
                            Clear_CMD_Status();
                            // Debug message
                            {
                                
                            }
//                            {
//                                char str[16];
//                                int  len;
//                                len = itoa_10(bIrTimeIndexIn_Output, str);
//                                str[len++] = ' ';
//                                len += itoa_10(bIrTimeIndexOut_Output, (str+len));
//                                str[len++] = '\n';
//                                VirtualSerial_MultiByteToHost(str, (uint16_t) len);
//                                USB_task_in_main_loop();
//                            }
                            break;

                        default:
                            break;
                    }
                }
                else
                {
                    // Debug message
                    {
                        uart_output_enqueue('X');
                        uart_output_enqueue('\n');
                    }
                }
                Reset_CheckSum();
                }
            }
        

        
/*        
      // For testing UART Rx-Tx: read then write        
      while(!uart_input_queue_empty_status())
      {
        if(!uart_output_queue_full_status())   // fill Tx Buffer until either Rx full is empty or Tx buffer is full
        {
          uint8_t value;
          value = uart_input_dequeue();// already check input_queue is not empty in advance
          uart_output_enqueue(value);  // already check output_queue is not full in advance
          UART0->INTEN |= UART_INTEN_THREIEN_Msk;   // Enable Tx interrupt to consume output buffer
        }
        else
        {
          break;
        }
      }
*/
    }                
  }


