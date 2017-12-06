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

extern void WDT_MySetup(void);
extern void WDT_MyClearTimeOutIntFlag(void);

void PWM0_IRQHandler(void)
{
	
    // Clear channel 0 period interrupt flag
    PWM_ClearIntFlag(PWM0, 0);
}

void WDT_IRQHandler(void)
{
    Reset_IR_Tx_running_status();
    // To trap Watch-dog timer when necessary
    WDT_ClearResetFlag();
}

void ACMP_IRQHandler(void)
{
    uint8_t compare_result;
    compare_result = ACMP_GET_OUTPUT(ACMP,1);
    ACMP_CLR_INT_FLAG(ACMP,1);
}

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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(ACMP_MODULE);	

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0CH01SEL_HCLK, 0);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HCLK, 0);

    // Select WDT clock
    CLK_SetModuleClock(WDT_MODULE,CLK_CLKSEL1_WDTSEL_LIRC,0);  // Use 10K clock
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */ // PB8 & PB9
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

    /* Set GPA multi-function pins for PWM0 Channel0 */ // PA12 & PB4 (PB7 is temporary)
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA12MFP_Msk) ) | SYS_GPA_MFP_PA12MFP_PWM0CH0;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk) ) | SYS_GPB_MFP_PB4MFP_PWM0CH0_INV;
    //SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk) ) | SYS_GPB_MFP_PB4MFP_GPIO;
    
    // Setup I2C on PB2/PB3
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk) ) | SYS_GPB_MFP_PB2MFP_I2C_SCL;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB3MFP_Msk) ) | SYS_GPB_MFP_PB3MFP_I2C_SDA;
    
    // Setup PB6 as comparator
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB6MFP_Msk) ) | SYS_GPB_MFP_PB6MFP_CMP6;
    
    // Setup other pins as GPIO
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA1MFP_Msk) ) | SYS_GPA_MFP_PA1MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA2MFP_Msk) ) | SYS_GPA_MFP_PA2MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA3MFP_Msk) ) | SYS_GPA_MFP_PA3MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA4MFP_Msk) ) | SYS_GPA_MFP_PA4MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA5MFP_Msk) ) | SYS_GPA_MFP_PA5MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA6MFP_Msk) ) | SYS_GPA_MFP_PA6MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA7MFP_Msk) ) | SYS_GPA_MFP_PA7MFP_GPIO;
    // PB8 UART
    // PB9 UART
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA10MFP_Msk) ) | SYS_GPA_MFP_PA10MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA11MFP_Msk) ) | SYS_GPA_MFP_PA11MFP_GPIO;                                                                                                                                                                                                                                                           
    // PA12 PWM0
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA13MFP_Msk) ) | SYS_GPA_MFP_PA13MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA14MFP_Msk) ) | SYS_GPA_MFP_PA14MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA15MFP_Msk) ) | SYS_GPA_MFP_PA15MFP_GPIO;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk) ) | SYS_GPB_MFP_PB0MFP_GPIO;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB1MFP_Msk) ) | SYS_GPB_MFP_PB1MFP_GPIO;
    // PB2 I2C
    // PB3 I2C
    // PB4 PWM
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB5MFP_Msk) ) | SYS_GPB_MFP_PB5MFP_GPIO;
    // PB6 comparator
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB7MFP_Msk) ) | SYS_GPB_MFP_PB7MFP_GPIO;
 
    /* Reset PWM0 channel0~channel3 */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(UART0_RST);
    SYS_ResetModule(I2C0_RST);
    SYS_ResetModule(TMR0_RST);
    SYS_ResetModule(ACMP_RST);

    /* Lock protected registers */
    SYS_LockReg();
}

void ProcessInputCommand(void)
{
    uint8_t repeat_cnt;
    switch(Read_CMD_Status())
    {
        case ENUM_CMD_STOP_CMD_RECEIVED:
            uart_output_enqueue_with_newline('S');
            Set_IR_Repeat_Cnt(0);
            Clear_CMD_Status();
            while(Get_IR_Tx_running_status()) {}        // Wait until previous Tx Finish
            Init_Parser();
            Init_Timer_App();
            Init_IR_buffer();
            break;

        case ENUM_CMD_REPEAT_COUNT_RECEIVED:
            repeat_cnt = Next_Repeat_Count_Get();
            Next_Repeat_Count_Set(0);
            if(repeat_cnt>=16)
            {
                uart_output_enqueue_with_newline('>');
            }
            else if (repeat_cnt>=10)
            {
                uart_output_enqueue_with_newline(repeat_cnt+'a'-10);
            }
            else
            {
                uart_output_enqueue_with_newline(repeat_cnt+'0');
            }
            if(repeat_cnt>0)
            {
                Set_IR_Repeat_Cnt(Get_IR_Repeat_Cnt()+repeat_cnt);
                IR_Transmit_Buffer_StartSend();
            }
            Clear_CMD_Status();
            break;
            
        case ENUM_CMD_INPUT_CMD_RECEIVED:
            if(Next_Command_Get()>=0xf0)
            {
                uart_output_enqueue_with_newline('U');
            }
            else 
            {
                if(Next_Command_Get()>=0xe0)
                {
                    uart_output_enqueue('V');
                }
                else if(Next_Command_Get()>=0xd0)
                {
                    uart_output_enqueue('W');
                }
                else if(Next_Command_Get()>=0xc0)
                {
                    uart_output_enqueue('Y');
                }
                {
                    char    temp_str[12+1];
                    int     temp_index, temp_length;
                    temp_length = itoa_10(Next_Input_Parameter_Get(),temp_str);
                    for (temp_index=0;temp_index<temp_length;temp_index++)
                        uart_output_enqueue(temp_str[temp_index]);
                    uart_output_enqueue('\n');
                }
            }
            Clear_CMD_Status();
            break;

        case ENUM_CMD_WIDTH_DATA_READY:
            uart_output_enqueue_with_newline('R');
            Clear_CMD_Status();
            Set_IR_Repeat_Cnt(0);
            while(Get_IR_Tx_running_status()) {}        // Wait until previous Tx Finish
            Set_IR_Repeat_Cnt(Next_Repeat_Count_Get());
            Set_PWM_period(Next_PWM_Period_Get());
            Set_PWM_duty_cycle(Next_DutyCycle_Period_Get());
            Copy_Input_Data_to_Tx_Data_and_Start();
            //IR_Transmit_Buffer_StartSend();
            // Debug message
            {
            }
            break;

        default:
            break;
    }
}

/* Main */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    UART_init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf(  "-----------------------------\n");
    printf(  " Warm greeting by BlueRat\n");
    printf(  " "__DATE__  "\n" );
    printf(  " "__TIME__ "\n" );
    printf(  "-----------------------------\n");
	
    Initialize_buffer();
    Init_Parser();    
    Init_Timer_App();

    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT3, GPIO_MODE_OUTPUT);
    
    /* set PWM0 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM0, PWM_CH0, 38000, 33); // Do not change 3rd/4th parameter because this function has been tailored to specific input parameter range
    PWM_EnableOutput(PWM0, 0x1);
        
    // Setup Timer
    Timer_Init();    

    // Setup Analog Comparator
    ACMP_Open(ACMP,1,ACMP_CMP1VNEG_VBG,0);
    ACMP_ENABLE_INT(ACMP,1);

    // Enable PWM channel 0 period interrupt
    //PWM0->INTEN = PWM_INTEN_PIEN0_Msk;
    NVIC_EnableIRQ(PWM0_IRQn);
    NVIC_EnableIRQ(TMR0_IRQn);	
    NVIC_EnableIRQ(ACMP_IRQn);
#ifdef ENABLE_WATCH_DOG_TIMER
    // Setup Watch Dog Timer
    WDT_MySetup();
    NVIC_EnableIRQ(WDT_IRQn);
#endif // ENABLE_WATCH_DOG_TIMER
    
    while(1)
    {
        if(!uart_input_queue_empty_status())
        {
            ProcessInputChar(uart_input_dequeue());
            if(CheckSum_Ready()==true)
            {
                if (Read_CheckSum()==0)
                {
                    ProcessInputCommand();
                }
                else
                {
                    uart_output_enqueue_with_newline('?');       // Checksum error
                }
                Reset_CheckSum();
            }
        }
        if(Get_IR_Tx_Finish_status())
        {
            Clear_IR_Tx_Finish();
            uart_output_enqueue_with_newline('+');               // Tx finish one-time
        }
#ifdef ENABLE_WATCH_DOG_TIMER
        WDT_ResetCounter();
#endif // ENABLE_WATCH_DOG_TIMER
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

