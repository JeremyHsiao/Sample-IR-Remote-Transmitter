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

extern void WDT_MySetup(void);
extern void WDT_MyClearTimeOutIntFlag(void);

void PWM0_IRQHandler(void)
{
    // Clear channel 0 period interrupt flag
    PWM_ClearIntFlag(PWM0, 0);
    PWM_ClearIntFlag(PWM0, 1);
}

void WDT_IRQHandler(void)
{
    Reset_IR_Tx_running_status();
    // To trap Watch-dog timer when necessary
    WDT_ClearResetFlag();
}

uint8_t compare_result;

void ACMP_IRQHandler(void)
{
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
    // Setup GPIO PA0/PA7
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA1MFP_Msk) ) | SYS_GPA_MFP_PA1MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA2MFP_Msk) ) | SYS_GPA_MFP_PA2MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA3MFP_Msk) ) | SYS_GPA_MFP_PA3MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA4MFP_Msk) ) | SYS_GPA_MFP_PA4MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA5MFP_Msk) ) | SYS_GPA_MFP_PA5MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA6MFP_Msk) ) | SYS_GPA_MFP_PA6MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA7MFP_Msk) ) | SYS_GPA_MFP_PA7MFP_GPIO;

    /* Set GPA multi-function pins for UART0 RXD and TXD */ // PA8 & PA9
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

    // GPIO PA 10/11
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA10MFP_Msk) ) | SYS_GPA_MFP_PA10MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA11MFP_Msk) ) | SYS_GPA_MFP_PA11MFP_GPIO;                                                                                                                                                                                                                                                           

    /* Set GPA multi-function pins for PWM0/1 Channel */ // PA12 & P13
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA12MFP_Msk) ) | SYS_GPA_MFP_PA12MFP_PWM0CH0;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA13MFP_Msk) ) | SYS_GPA_MFP_PA13MFP_PWM0CH1;

    // GPIO PA 14/15
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA14MFP_Msk) ) | SYS_GPA_MFP_PA14MFP_GPIO;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA15MFP_Msk) ) | SYS_GPA_MFP_PA15MFP_GPIO;                                                                                                                                                                                                                                                           

    // GPIO input PB0/1
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk) ) | SYS_GPB_MFP_PB0MFP_GPIO;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB1MFP_Msk) ) | SYS_GPB_MFP_PB1MFP_GPIO;

     // Setup I2C on PB2/PB3
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk) ) | SYS_GPB_MFP_PB2MFP_I2C_SCL;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB3MFP_Msk) ) | SYS_GPB_MFP_PB3MFP_I2C_SDA;
    
    /* Set GPB multi-function pins for PWM0/1 Channel Inverted output */ // PB4 & PB5
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk) ) | SYS_GPB_MFP_PB4MFP_PWM0CH0_INV;
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB5MFP_Msk) ) | SYS_GPB_MFP_PB5MFP_PWM0CH1_INV;
    
    // Setup PB6 as comparator
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB6MFP_Msk) ) | SYS_GPB_MFP_PB6MFP_CMP6;

    // GPIO PB7
    SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB7MFP_Msk) ) | SYS_GPB_MFP_PB7MFP_GPIO;

    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT6, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT7, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI);
    GPIO_SetMode(PA, BIT11, GPIO_MODE_QUASI);
    GPIO_SetMode(PA, BIT14, GPIO_MODE_QUASI);
    GPIO_SetMode(PA, BIT15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT1, GPIO_MODE_QUASI);
    //GPIO_SetMode(PB, BIT7, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
    // Set output as 1 for all Quasi-bidirectional mode GPIO
    PA->DATMSK = ~(GPIO_DOUT_DOUT10_Msk|GPIO_DOUT_DOUT11_Msk|GPIO_DOUT_DOUT14_Msk|GPIO_DOUT_DOUT15_Msk);
    PA->DOUT = (GPIO_DOUT_DOUT10_Msk|GPIO_DOUT_DOUT11_Msk|GPIO_DOUT_DOUT14_Msk|GPIO_DOUT_DOUT15_Msk);
    PB->DATMSK = ~(GPIO_DOUT_DOUT7_Msk|GPIO_DOUT_DOUT1_Msk|GPIO_DOUT_DOUT0_Msk);
    PB->DOUT = (GPIO_DOUT_DOUT1_Msk|GPIO_DOUT_DOUT0_Msk);
    //PB->DOUT = (GPIO_DOUT_DOUT7_Msk|GPIO_DOUT_DOUT1_Msk|GPIO_DOUT_DOUT0_Msk);

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
    switch(Next_Command_Get())
    {
        // STOP ALL
        case ENUM_CMD_STOP_ALL:
            uart_output_enqueue_with_newline('Z');
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
                    uint32_t input_data, temp_pa, temp_pb;
                    temp_pa = PA->PIN;
                    temp_pb = PB->PIN;
                    temp_pb &= 0x82; // keep PB7/PB1
                    if(temp_pb&0x80)
                    {
                        input_data = (temp_pb & 0x2) | 0x04;
                    }
                    input_data = temp_pb>>1;
                    temp_pa &= 0xcc00; // keep PA15/PA14/PA11/PA10
                    input_data = (input_data<<4) | ((temp_pa>>10)&0x03) | ((temp_pa>>12)&0x0c);
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
                        uart_output_enqueue('P');
                        OutputHexValue(output_data);
                        uart_output_enqueue('\n');
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
                uart_output_enqueue('p');
                OutputHexValue(return_data);
                uart_output_enqueue('\n');
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
                uart_output_enqueue('R');
                OutputHexValue(output_data);
                uart_output_enqueue('\n');
            }
            break;

        case ENUM_CMD_INPUT_TX_SIGNAL:
            
            uart_output_enqueue_with_newline('T');
            Set_IR_Repeat_Cnt(0);
            while(Get_IR_Tx_running_status()) {}        // Wait until previous Tx Finish
            Set_IR_Repeat_Cnt(Next_Repeat_Count_Get());
            Set_PWM_period(Next_PWM_Period_Get());
            Set_PWM_duty_cycle(Next_DutyCycle_Period_Get());
            Copy_Input_Data_to_Tx_Data_and_Start();
            break;

        default:
            uart_output_enqueue_with_newline('U');
            OutputHexValue(Next_Command_Get());
            uart_output_enqueue('\n');
            break;
    }
}

void CheckIfISP(void)
{
    if(((PB->PIN)&0x1)==0) 
    {
        uint32_t   au32Config[1] = { 0xffffffff };

//        FMC_Open();
//        
//        // Read Config
//        if (FMC_ReadConfig(au32Config, 1) < 0) {
//            printf("\nRead User Config failed!\n");
//            return;
//        }

//        if ((au32Config[0] & (1UL<<1))==0)
//        {
//            printf("\nSecurity Bit is locked thus cannot update application!\n");
//            return;
//        }

//        //
//        if ( (au32Config[0]&(1UL<<7))==0 ) 
//        {
//            printf("\nNo need to update config bit!\n");
//            return;
//        }
        
        // Enable LDROM and boot-from-LDROM then Write Config
        FMC_EnableConfigUpdate();
        au32Config[0] &= ~(1UL<<7);      // 0: boot from LDROM
        if (FMC_WriteConfig(au32Config, 1) < 0)
            return;

        FMC_DisableConfigUpdate();
//        FMC_Close();
        
        // Perform chip reset to make new User Config take effect
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
}

/* Main */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    CheckIfISP();

    UART_init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf(  "-----------------------------\n");
    printf(  " Warm greeting by BlueRat v02\n");
    printf(  " "__DATE__  "\n" );
    printf(  " "__TIME__ "\n" );
    printf(  "-----------------------------\n");
	
    Initialize_buffer();
    Init_Parser();    
    Init_Timer_App();

    /* set PWM0 channel 0 output configuration */
    PWM_ConfigOutputChannel_v2(PWM0, 38000, 33); // Do not change 3rd/4th parameter because this function has been tailored to specific input parameter range
    PWM_EnableOutput(PWM0, 0x3); // Enable Output of both Channels at once
    // PWM interrup not used at this moment
    //PWM_EnableInt(PWM0, 0x0, 1);
    //PWM_EnableInt(PWM0, 0x1, 1);
    PWM_DisableInt(PWM0, 0x0);
    PWM_DisableInt(PWM0, 0x1);
        
    // Setup Timer
    Timer_Init();    

    // Setup Analog Comparator
    ACMP_Open(ACMP,1,ACMP_CMP1VNEG_VBG,0);
    ACMP_ENABLE_INT(ACMP,1);

    //NVIC_EnableIRQ(PWM0_IRQn);   // PWM interrup not used at this moment
    NVIC_DisableIRQ(PWM0_IRQn);
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
                Next_Command_Clear();
            }
        }
        
        if(Get_IR_Tx_Finish_status())
        {
            Clear_IR_Tx_Finish();
            uart_output_enqueue_with_newline('+');               // Tx finish one-time
        }
        if(Get_IR_Tx_Finish_All_status())
        {
            Clear_IR_Tx_All_Finish();
            uart_output_enqueue_with_newline('S');               // Tx finish and currently no more to send
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

