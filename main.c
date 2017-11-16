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

/* Buffer size, this buffer for uart receive & send data. */
#define UART_BUF_SIZE      64
uint8_t u8Buffer_IN[UART_BUF_SIZE];
uint8_t *UART_BUF_IN_WRITE_PTR = u8Buffer_IN;
uint8_t *UART_BUF_IN_REAR_PTR = u8Buffer_IN;

void UART0_IRQHandler(void)
{
	if(UART_IS_RX_READY(UART0))
  {  
    while(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
    {
      *UART_BUF_IN_WRITE_PTR = UART0->DAT;               /* Get Data from UART RX  */
      UART_BUF_IN_WRITE_PTR++;
      if(UART_BUF_IN_WRITE_PTR>=(u8Buffer_IN+UART_BUF_SIZE))
      {
        UART_BUF_IN_WRITE_PTR = u8Buffer_IN;
      }
      if(UART_BUF_IN_WRITE_PTR==UART_BUF_IN_REAR_PTR)
      {
        //UART_BUF_IN_FULL = 1;
        //while(1){} // trap here for debug purpose
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
	
	while(1)
	{
    if(UART_BUF_IN_WRITE_PTR!=UART_BUF_IN_REAR_PTR)
    {
      UART_Write(UART0,UART_BUF_IN_REAR_PTR,1);
      UART_BUF_IN_REAR_PTR++;
      if(UART_BUF_IN_REAR_PTR>=(u8Buffer_IN+UART_BUF_SIZE))
      {
        UART_BUF_IN_REAR_PTR = u8Buffer_IN;
      }
    }	}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
