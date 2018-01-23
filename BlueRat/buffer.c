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
#include "timer_app.h"

// helper function
// Clear Buffer
#define BUF_CLEAR(WRITE_PTR, REAR_PTR, BUFFER_START_PTR, STATUS_BIT) {  WRITE_PTR=BUFFER_START_PTR; REAR_PTR=BUFFER_START_PTR; STATUS_BIT=0; }
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
#define UART_RX_BUF_SIZE      24
uint8_t u8Buffer_RX[UART_RX_BUF_SIZE];
uint8_t *UART_BUF_RX_WRITE_PTR = u8Buffer_RX;
uint8_t *UART_BUF_RX_REAR_PTR = u8Buffer_RX;
uint8_t UART_BUF_RX_FULL = 0;

// UART-TX Buffer
#define UART_TX_BUF_SIZE      32
uint8_t u8Buffer_TX[UART_TX_BUF_SIZE];
uint8_t *UART_BUF_TX_WRITE_PTR = u8Buffer_TX;
uint8_t *UART_BUF_TX_REAR_PTR = u8Buffer_TX;
uint8_t UART_BUF_TX_FULL = 0;

// Store inputing IR data from UART    
// IR-Data Array
#define IR_DATA_BUF_SIZE      256
uint32_t u32Buffer_IR_DATA_Width[IR_DATA_BUF_SIZE];
uint32_t *IR_BUF_DATA_WRITE_PTR = u32Buffer_IR_DATA_Width;

// Use as data for Tx output
// IR-pulse-TX Array
#define IR_TX_BUF_SIZE      IR_DATA_BUF_SIZE
uint32_t u32Buffer_IR_TX_Width[IR_TX_BUF_SIZE];
uint32_t *IR_BUF_TX_WRITE_PTR =u32Buffer_IR_TX_Width;
uint32_t *IR_BUF_TX_REAR_PTR =u32Buffer_IR_TX_Width;
uint8_t IR_BUF_TX_FULL = 0;

// New Tx mechanism -- try to use solely PWM
// IR-PWM-Pulse-Array
#define IR_PWM_BUF_SIZE      (IR_DATA_BUF_SIZE)
T_PWM_BUFFER T_PWM_BUFFER_Buf[IR_PWM_BUF_SIZE];
T_PWM_BUFFER *PWM_BUF_WRITE_PTR =T_PWM_BUFFER_Buf;
T_PWM_BUFFER *PWM_BUF_READ_PTR =T_PWM_BUFFER_Buf;
uint8_t PWM_BUF_FULL = 0;

//
// Common function
//

void Init_IR_buffer(void)
{
  BUF_CLEAR(IR_BUF_TX_WRITE_PTR, IR_BUF_TX_REAR_PTR, u32Buffer_IR_TX_Width, IR_BUF_TX_FULL);
  //  
  BUF_CLEAR(IR_BUF_TX_WRITE_PTR, IR_BUF_TX_WRITE_PTR, u32Buffer_IR_DATA_Width, IR_BUF_TX_FULL);
}

void Initialize_buffer(void)
{
  BUF_CLEAR(UART_BUF_RX_WRITE_PTR, UART_BUF_RX_REAR_PTR, u8Buffer_RX, UART_BUF_RX_FULL);
  BUF_CLEAR(UART_BUF_TX_WRITE_PTR, UART_BUF_TX_REAR_PTR, u8Buffer_TX, UART_BUF_TX_FULL);
  Init_IR_buffer();
}

//
// buffer function for UART-Input
//

uint8_t uart_input_queue_empty_status(void)
{
  return (UART_BUF_RX_WRITE_PTR==UART_BUF_RX_REAR_PTR)?TRUE:FALSE;
}

uint8_t uart_input_queue_full_status(void)
{
  return (UART_BUF_RX_FULL);
}

//
// This function is currently only used in UART Rx-interrupt
//
uint8_t uart_input_enqueue(uint8_t input_data)
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

uint8_t uart_input_dequeue(void)
{
  uint8_t   return_value;

  return_value = *UART_BUF_RX_REAR_PTR;
  if(!(UART_BUF_RX_WRITE_PTR==UART_BUF_RX_REAR_PTR))
  {
    BUF_PTR_INCREASE(UART_BUF_RX_REAR_PTR,u8Buffer_RX,UART_RX_BUF_SIZE);
    UART0->INTEN &= ~UART_INTEN_RDAIEN_Msk; // update buffer-full status in the block where UART-RX interrupt is diaabled temporarily.sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
    BUF_FULL_CHECK(UART_BUF_RX_WRITE_PTR,UART_BUF_RX_REAR_PTR,UART_RX_BUF_SIZE,UART_BUF_RX_FULL);  
    UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
  }
  return return_value;
}

//
// buffer function for UART-Output
//

uint8_t uart_output_queue_empty_status(void)
{
  return (UART_BUF_TX_WRITE_PTR==UART_BUF_TX_REAR_PTR)?TRUE:FALSE;
}

uint8_t uart_output_queue_full_status(void)
{
  return (UART_BUF_TX_FULL);
}

uint8_t uart_output_enqueue(uint8_t input_data)
{
  // if output_queue is empty now, we can put data to Tx directly  
  if((!(UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk))&&(uart_output_queue_empty_status()))
  {
    UART0->DAT = input_data;
    return 1;
  } 
  else if(!UART_BUF_TX_FULL)  // must check a buffer-full status before an "add"
  {
    UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;   // access queue so UART-TX interrupt (where it will dequeue) is disabled temporarily.  
    *UART_BUF_TX_WRITE_PTR = input_data;
    BUF_PTR_INCREASE(UART_BUF_TX_WRITE_PTR,u8Buffer_TX,UART_TX_BUF_SIZE);
    BUF_FULL_CHECK(UART_BUF_TX_WRITE_PTR,UART_BUF_TX_REAR_PTR,UART_TX_BUF_SIZE, UART_BUF_TX_FULL); // always update buffer-full status at the end of an "add"
    UART0->INTEN |= UART_INTEN_THREIEN_Msk;   // Enable Tx interrupt to consume output buffer  
    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t uart_output_enqueue_with_newline (uint8_t input_data)
{
    if(uart_output_enqueue(input_data))
    {
        if(uart_output_enqueue('\n'))
        {
            return (2);
        }
        else
        {
            return (1);
        }
    }
    else
    {
        return(0);
    }
}

//
// This function is currently only used in UART Tx-interrupt
//
uint8_t uart_output_dequeue(void)
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

//
// IR-pulse-DATA Array function
//
void IR_data_restart_write_pointer(void)
{
  IR_BUF_DATA_WRITE_PTR = u32Buffer_IR_DATA_Width;
}

uint8_t IR_data_full(void)
{
    return (IR_BUF_DATA_WRITE_PTR>=(u32Buffer_IR_DATA_Width+IR_DATA_BUF_SIZE))? TRUE: FALSE;
}

uint8_t IR_data_add(uint32_t input_data)
{
  if(!IR_data_full())  // must check a buffer-full status before an "add"
  {
    *IR_BUF_DATA_WRITE_PTR = input_data;
    IR_BUF_DATA_WRITE_PTR++;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

void Copy_Input_Data_to_Tx_Data_and_Start(void)
{
    uint32_t *src = u32Buffer_IR_DATA_Width, *end = IR_BUF_DATA_WRITE_PTR;
    IR_BUF_TX_WRITE_PTR = u32Buffer_IR_TX_Width;
    if(src<end)
    {
        *IR_BUF_TX_WRITE_PTR++ = *src++;
        IR_Transmit_Buffer_StartSend();
        while(src<end)                   // from u32Buffer_IR_DATA_Width to IR_BUF_DATA_WRITE_PTR
        {
            *IR_BUF_TX_WRITE_PTR++ = *src++;
        }
    }
}

//
// IR-pulse-TX Array function
//

void IR_output_restart_read_pointer(void)
{
  IR_BUF_TX_REAR_PTR = u32Buffer_IR_TX_Width;
}

uint8_t IR_output_end_of_data(void)
{
    return (IR_BUF_TX_REAR_PTR==IR_BUF_TX_WRITE_PTR)? TRUE: FALSE;
}

uint8_t IR_output_read(uint32_t *return_value_ptr)
{
  if(IR_BUF_TX_REAR_PTR<IR_BUF_TX_WRITE_PTR)
  {
    *return_value_ptr = *IR_BUF_TX_REAR_PTR;
    IR_BUF_TX_REAR_PTR++;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

//
// PWM-pulse Array function
//
void PWM_Pulse_restart_read_pointer(void)
{
  PWM_BUF_READ_PTR = T_PWM_BUFFER_Buf;
}

uint8_t PWM_Pulse_end_of_data(void)
{
    return (PWM_BUF_READ_PTR==PWM_BUF_WRITE_PTR)? TRUE: FALSE;
}

uint8_t PWM_Pulse_read(T_PWM_BUFFER *return_value_ptr)
{
  if(PWM_BUF_READ_PTR<PWM_BUF_WRITE_PTR)
  {
    *return_value_ptr = *PWM_BUF_READ_PTR;
    PWM_BUF_READ_PTR++;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}
