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
#define UART_RX_BUF_SIZE      64
#define UART_TX_BUF_SIZE      64
// RX Buffer
uint8_t u8Buffer_RX[UART_RX_BUF_SIZE];
uint8_t *UART_BUF_RX_WRITE_PTR = u8Buffer_RX;
uint8_t *UART_BUF_RX_REAR_PTR = u8Buffer_RX;
uint8_t UART_BUF_RX_FULL = 0;
#define CHECK_RX_BUF_EMPTY() (UART_BUF_RX_WRITE_PTR==UART_BUF_RX_REAR_PTR)
// TX Buffer
uint8_t u8Buffer_TX[UART_TX_BUF_SIZE];
uint8_t *UART_BUF_TX_WRITE_PTR = u8Buffer_TX;
uint8_t *UART_BUF_TX_REAR_PTR = u8Buffer_TX;
uint8_t UART_BUF_TX_FULL = 0;
#define CHECK_TX_BUF_EMPTY() (UART_BUF_TX_WRITE_PTR==UART_BUF_TX_REAR_PTR)
//

// helper function
// Clear Buffer
#define BUF_CLEAR(WRITE_PTR, REAR_PTR, BUFFER_START_PTR, STATUS_BIT) {  WRITE_PTR=REAR_PTR=BUFFER_START_PTR; STATUS_BIT=0; }
// Increase Ring Buffer Pointer by 1 and back to buffer starting point when out of bound
#define BUF_PTR_INCREASE(PTR, BUFFER_START_PTR, BUFFER_SIZE)                        \
    {                                                                               \
        PTR++;                                                                      \
        if(PTR>=(BUFFER_START_PTR+BUFFER_SIZE))                                     \
        {                                                                           \
          PTR = BUFFER_START_PTR;                                                   \
        }                                                                           \
    }                                                                               
// Check if Rx Buffer is full and not able to insert more at this moment 
    #define BUF_FULL_CHECK(WRITE_PTR, REAR_PTR, BUF_SIZE, STATUS_BIT)                             \
    {                                                                               \
      if(REAR_PTR>WRITE_PTR)     /* ....write.....rear..... */                      \
      {                                                                             \
        if((WRITE_PTR+1)==REAR_PTR)                                                 \
        {                                                                           \
          STATUS_BIT = 1;                                                           \
          break;                                                                    \
        }                                                                           \
        else                                                                        \
        {                                                                           \
          STATUS_BIT = 0;                                                           \
        }                                                                           \
      }                                                                             \
      else if(WRITE_PTR>REAR_PTR)     /* ....rear.....write.....*/                  \
      {                                                                             \
        if((REAR_PTR+BUF_SIZE-1)==WRITE_PTR)                                   \
        {                                                                           \
          STATUS_BIT = 1;                                                           \
          break;                                                                    \
        }                                                                           \
        else                                                                        \
        {                                                                           \
          STATUS_BIT = 0;                                                           \
        }                                                                           \
      }                                                                             \
      else                                                                          \
      {                                                                             \
          STATUS_BIT = 0;                                                           \
      }                                                                             \
    }                                                                               


void UART0_IRQHandler(void)
{
  // Tx when FIFO is empty - interrupt occurs less than using FIFO is not full 
  if (UART_IS_TX_EMPTY(UART0))
  {
    do 
    {
      if(!CHECK_TX_BUF_EMPTY())
      {
        UART0->DAT = *UART_BUF_TX_REAR_PTR;
        BUF_PTR_INCREASE(UART_BUF_TX_REAR_PTR,u8Buffer_TX,UART_TX_BUF_SIZE);
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
      if (!UART_BUF_RX_FULL)
      {
        *UART_BUF_RX_WRITE_PTR = UART0->DAT;               /* Store Data into UART RX  */
        BUF_PTR_INCREASE(UART_BUF_RX_WRITE_PTR,u8Buffer_RX,UART_RX_BUF_SIZE);
        BUF_FULL_CHECK(UART_BUF_RX_WRITE_PTR,UART_BUF_RX_REAR_PTR,UART_RX_BUF_SIZE,UART_BUF_RX_FULL);
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
	
    BUF_CLEAR(UART_BUF_RX_WRITE_PTR, UART_BUF_RX_REAR_PTR, u8Buffer_RX, UART_BUF_RX_FULL);
    BUF_CLEAR(UART_BUF_TX_WRITE_PTR, UART_BUF_TX_REAR_PTR, u8Buffer_TX, UART_BUF_TX_FULL);
  
	while(1)
	{
    while(!CHECK_RX_BUF_EMPTY())
    {
      if(!UART_BUF_TX_FULL)   // fill Tx Buffer until either Rx full is empty or Tx buffer is full
      {
        // Copy data from RX_BUF to TX_BUF
        *UART_BUF_TX_WRITE_PTR = *UART_BUF_RX_REAR_PTR;         
        // RX Read_pointer++
        BUF_PTR_INCREASE(UART_BUF_RX_REAR_PTR,u8Buffer_RX,UART_RX_BUF_SIZE);
        // TX write_pointer++
        BUF_PTR_INCREASE(UART_BUF_TX_WRITE_PTR,u8Buffer_TX,UART_TX_BUF_SIZE);
        // Check if Tx Buffer is full --> UART_BUF_TX_FULL=1 when full
        BUF_FULL_CHECK(UART_BUF_TX_WRITE_PTR,UART_BUF_TX_REAR_PTR,UART_TX_BUF_SIZE, UART_BUF_TX_FULL);
      }
      UART0->INTEN |= UART_INTEN_THREIEN_Msk;   // Enable Tx interrupt to consume output buffer
    }
  }                
}


