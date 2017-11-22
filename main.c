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

void UART0_IRQHandler(void)
{
  // Tx when FIFO is empty - interrupt occurs less than using FIFO is not full 
  if (UART_IS_TX_EMPTY_INT(UART0))
  {
    do 
    {
      if(!uart_output_buffer_empty_status())
      {
        UART0->DAT = uart_read_output_buffer();
      }
      else
      {
        UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        break;       // No more data to Tx output
      }
    }
    while(!(UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));   /* If Tx is not full */
  }
  // Rx when FIFO is ready - interrupt occurs more than using FIFO is full so that we can prevent IRQ blocking causiing FIFO data overwritten
	if(UART_IS_RX_READY(UART0))
  {  
    while(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
    {
      if (!uart_input_buffer_full_status())
      {
        uart_add_input_buffer(UART0->DAT);
      }
      else
      {
        break;
      }
    }
	}	
}

void UART_init(void)
{
  //CLK_EnableModuleClock(UART_MODULE); 
	//UART_Open(UART0, 115200);	
	UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
  //UART0->INTEN |= UART_INTEN_THREIEN_Msk;
	NVIC_EnableIRQ(UART0_IRQn);
	//SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	//SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;	
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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
	  SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	  SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

    /* Lock protected registers */
    SYS_LockReg();
}

/* Main */
int main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    UART_init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                      My Sample                             |\n");
    printf("+------------------------------------------------------------------------+\n");
	  printf("Press any key to test.\n");
	
    Initialize_buffer();
 
    while(1)
    {
      if(!uart_input_buffer_empty_status())
      {
        if(!uart_output_buffer_full_status())   // fill Tx Buffer until either Rx full is empty or Tx buffer is full
        {
          if(uart_add_output_buffer(uart_read_input_buffer()))
          {
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;   // Enable Tx interrupt to consume output buffer
          }
        }
      }
    }                
  }


