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
 
//
// External Function declaration
//     
extern void Initialize_buffer(void);

extern uint8_t uart_input_buffer_empty_status(void);    
extern uint8_t uart_input_buffer_full_status(void);
extern uint8_t uart_add_input_buffer(uint8_t input_data);
extern uint8_t uart_read_input_buffer(void);

extern uint8_t uart_output_buffer_empty_status(void);
extern uint8_t uart_output_buffer_full_status(void);
extern uint8_t uart_add_output_buffer(uint8_t input_data);
extern uint8_t uart_read_output_buffer(void);

extern uint8_t IR_output_buffer_empty_status(void);
extern uint8_t IR_output_buffer_full_status(void);
extern uint8_t IR_add_output_buffer(uint32_t input_data);
extern uint32_t IR_read_output_buffer(void);
