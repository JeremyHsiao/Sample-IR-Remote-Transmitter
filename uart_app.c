/**************************************************************************//**
 * @file     uart_app.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/11/23 11:55p $
 * @brief    Uart code - non-driver
 *
 * @note
 *
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "buffer.h"
#include "uart_app.h"

int itoa_10(uint32_t value, char* result)
{
	// check that the base if valid

	char*       ptr = result, *ptr1 = result, tmp_char;
	uint32_t    tmp_value;
    int         str_len;

	str_len = 0;
	do {
		tmp_value = value % 10;
		value /= 10;
		*ptr++ = "0123456789" [tmp_value];
		str_len++;
	} while ( value );

	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return str_len;
}

void UART0_IRQHandler(void)
{
  // Tx when FIFO is empty - interrupt occurs less than using FIFO is not full
  if (UART_IS_TX_EMPTY_INT(UART0))
  {
    do
    {
      if(!uart_output_queue_empty_status())
      {
        UART0->DAT = uart_output_dequeue();
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
      if (!uart_input_queue_full_status())
      {
        uart_input_enqueue(UART0->DAT);
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
