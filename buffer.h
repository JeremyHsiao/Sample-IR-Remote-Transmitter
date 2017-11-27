/****************************************************************************
 * @file     buffer.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 2017/11/22 10:00p $
 * @brief    Header file for ring-buffer function - for UART & for output pulse
 *
 * @note
 * 
 *
 ******************************************************************************/
#ifndef _BUFFER_H_
#define _BUFFER_H_

//
// External Function declaration
//     
extern void Initialize_buffer(void);

extern uint8_t uart_input_queue_empty_status(void);    
extern uint8_t uart_input_queue_full_status(void);
extern uint8_t uart_input_enqueue(uint8_t input_data);
extern uint8_t uart_input_dequeue(void);

extern uint8_t uart_output_queue_empty_status(void);
extern uint8_t uart_output_queue_full_status(void);
extern uint8_t uart_output_enqueue(uint8_t input_data);
extern uint8_t uart_output_dequeue(void);

extern void IR_output_restart_write_pointer(void);
extern void IR_output_restart_read_pointer(void);
extern uint8_t IR_output_end_of_data(void);
extern uint8_t IR_output_add(uint32_t input_data);
extern uint8_t IR_output_read(uint32_t *return_value_ptr);

#endif /* !_BUFFER_H_ */
