/****************************************************************************
 * @file     buffer.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 2017/11/22 10:00p $
 * @brief    All kinds of ring-buffer function - for UART & for output pulse
 *
 * @note
 *
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "buffer.h"

 /* Buffer size, this buffer for uart receive & send data. */
#define UART_RX_BUF_SIZE      16
#define UART_TX_BUF_SIZE      16

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

//
// End of helper function
//

// UART-RX Buffer
uint8_t u8Buffer_RX[UART_RX_BUF_SIZE];
uint8_t *UART_BUF_RX_WRITE_PTR = u8Buffer_RX;
uint8_t *UART_BUF_RX_REAR_PTR = u8Buffer_RX;
uint8_t UART_BUF_RX_FULL = 0;

// UART-TX Buffer
uint8_t u8Buffer_TX[UART_TX_BUF_SIZE];
uint8_t *UART_BUF_TX_WRITE_PTR = u8Buffer_TX;
uint8_t *UART_BUF_TX_REAR_PTR = u8Buffer_TX;
uint8_t UART_BUF_TX_FULL = 0;

void Initialize_buffer(void)
{
  BUF_CLEAR(UART_BUF_RX_WRITE_PTR, UART_BUF_RX_REAR_PTR, u8Buffer_RX, UART_BUF_RX_FULL);
  BUF_CLEAR(UART_BUF_TX_WRITE_PTR, UART_BUF_TX_REAR_PTR, u8Buffer_TX, UART_BUF_TX_FULL);

}

//
// buffer function for UART-Input
//

uint8_t uart_input_buffer_empty_status(void)
{
  return (UART_BUF_RX_WRITE_PTR==UART_BUF_RX_REAR_PTR)?TRUE:FALSE;
}

uint8_t uart_input_buffer_full_status(void)
{
  return (UART_BUF_RX_FULL);
}

uint8_t uart_add_input_buffer(uint8_t input_data)
{
  if(!UART_BUF_RX_FULL)  // must check a buffer-full status before an "add"
  {
    *UART_BUF_RX_WRITE_PTR = input_data;
    BUF_PTR_INCREASE(UART_BUF_RX_WRITE_PTR,u8Buffer_RX,UART_RX_BUF_SIZE);
    BUF_FULL_CHECK(UART_BUF_RX_WRITE_PTR,UART_BUF_RX_REAR_PTR,UART_RX_BUF_SIZE,UART_BUF_RX_FULL); // always update buffer-full status at the end of an "add"
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

uint8_t uart_read_input_buffer(void)
{
  uint8_t   return_value;

  return_value = *UART_BUF_RX_REAR_PTR;
  if(!(UART_BUF_RX_WRITE_PTR==UART_BUF_RX_REAR_PTR))
  {
    BUF_PTR_INCREASE(UART_BUF_RX_REAR_PTR,u8Buffer_RX,UART_RX_BUF_SIZE);
    UART_BUF_RX_FULL = FALSE; // Clear buffer-full at the end of a "read"
  }
  return return_value;
}

//
// buffer function for UART-Output
//

uint8_t uart_output_buffer_empty_status(void)
{
  return (UART_BUF_TX_WRITE_PTR==UART_BUF_TX_REAR_PTR)?TRUE:FALSE;
}

uint8_t uart_output_buffer_full_status(void)
{
  return (UART_BUF_TX_FULL);
}

uint8_t uart_add_output_buffer(uint8_t input_data)
{
  if(!UART_BUF_TX_FULL)  // must check a buffer-full status before an "add"
  {
    *UART_BUF_TX_WRITE_PTR = input_data;
    BUF_PTR_INCREASE(UART_BUF_TX_WRITE_PTR,u8Buffer_TX,UART_TX_BUF_SIZE);
    BUF_FULL_CHECK(UART_BUF_TX_WRITE_PTR,UART_BUF_TX_REAR_PTR,UART_TX_BUF_SIZE, UART_BUF_TX_FULL); // always update buffer-full status at the end of an "add"
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

uint8_t uart_read_output_buffer(void)
{
  uint8_t   return_value;

  return_value = *UART_BUF_TX_REAR_PTR;
  if(!(UART_BUF_TX_WRITE_PTR==UART_BUF_TX_REAR_PTR))
  {
    BUF_PTR_INCREASE(UART_BUF_TX_REAR_PTR,u8Buffer_TX,UART_TX_BUF_SIZE);
    UART_BUF_TX_FULL = FALSE; // Clear buffer-full at the end of a "read"
  }
  return return_value;
}
