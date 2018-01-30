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
extern void Init_IR_buffer(void);

extern uint8_t uart_input_queue_empty_status(void);    
extern uint8_t uart_input_queue_full_status(void);
extern uint8_t uart_input_enqueue(uint8_t input_data);
extern uint8_t uart_input_dequeue(void);

extern uint8_t uart_output_queue_empty_status(void);
extern uint8_t uart_output_queue_full_status(void);
extern uint8_t uart_output_enqueue(uint8_t input_data);
extern uint8_t uart_output_enqueue_with_newline(uint8_t input_data);
extern uint8_t uart_output_dequeue(void);

extern void IR_data_restart_write_pointer(void);
extern uint8_t IR_data_full(void);
extern uint8_t IR_data_add(uint32_t input_data);
extern void Copy_Input_Data_to_Tx_Data_and_Start(void);


//extern void IR_output_restart_write_pointer(void);
extern void IR_output_restart_read_pointer(void);
extern uint8_t IR_output_end_of_data(void);
extern uint8_t IR_output_read(uint32_t *return_value_ptr);

// New Tx mechanism -- try to use solely PWM
// IR-PWM-Pulse-Array
typedef struct
{
    uint32_t repeat_no;
    uint32_t high_cnt;
    uint32_t low_cnt;
} T_PWM_BUFFER;
extern void PWM_Pulse_restart_read_pointer(void);
extern uint8_t PWM_Pulse_end_of_data(void);
extern uint8_t PWM_Pulse_read(T_PWM_BUFFER *return_value_ptr);
extern void Copy_Input_Data_to_PWM_Data_and_Start(void);

#endif /* !_BUFFER_H_ */
