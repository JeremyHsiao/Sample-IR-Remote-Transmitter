/*
 * parser.c
 *
 *  Created on: 2017年10月26日
 *      Author: Jeremy.Hsiao
 */

#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "buffer.h"
#include "parser.h"

typedef enum {
    ENUM_PARSING_STATE_WAIT_SYNC_BYTE = 0,				 // Also initial state
    ENUM_PARSING_STATE_WAIT_COMMAND_CODE,
    ENUM_PARSING_STATE_WAIT_REPEAT_COUNT,
    ENUM_PARSING_STATE_WAIT_PWN_DUTY_CYCLE,
    ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_HIGH,
    ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_LOW,
    ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_1ST_INPUT,
    ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_WORD_LOW,
    ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_2ND,
    ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_3RD,
    ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_4TH,
    ENUM_PARSING_STATE_WAIT_CHECKSUM,
    ENUM_PARSING_STATE_UNKNOWN_STATE,
    ENUM_PARSING_STATE_WAIT_DATA_BYTE,
    ENUM_PARSING_STATE_WAIT_DATA_WORD_HIGH,
    ENUM_PARSING_STATE_WAIT_DATA_WORD_LOW,
    ENUM_PARSING_STATE_WAIT_DATA_DWORD_1ST,
    ENUM_PARSING_STATE_WAIT_DATA_DWORD_2ND,
    ENUM_PARSING_STATE_WAIT_DATA_DWORD_3RD,
    ENUM_PARSING_STATE_WAIT_DATA_DWORD_4TH,
    ENUM_PARSING_STATE_WAIT_DATA_CHECKSUM,
    ENUM_PARSING_STATE_UNKNOWN_STATE_INPUT,
    ENUM_PARSING_STATE_MAX
}   ENUM_PARSING_STATE;

static uint32_t				Next_PWM_period;
static uint32_t				Next_PWM_duty_cycle;
static uint8_t      		Internal_CheckSum;
static uint8_t				Next_Repeat_Count;
static ENUM_PARSING_STATE	current_state;
//static ENUM_CMD_STATUS		current_cmd_status;
static Bool					CheckSum_Read;
static uint8_t              Next_Command;
static uint32_t             Next_input_parameter;

void Init_Parser(void)
{
    Next_PWM_period = (uint32_t) (48000000/(38000));		// For 38KHz PWM pulse -- To be updated
    Next_PWM_duty_cycle = 33; // 50%
    Next_Repeat_Count = 0;
    Internal_CheckSum = 0xff;
    current_state = ENUM_PARSING_STATE_WAIT_SYNC_BYTE; // Initial State
    //current_cmd_status = ENUM_CMD_IDLE;
    CheckSum_Read = false;
    Next_Command = 0xfe;
    Next_input_parameter = 0;
}

Bool CheckSum_Ready(void)
{
	return CheckSum_Read;
}

uint8_t Read_CheckSum(void)
{
	return Internal_CheckSum;
}

void Reset_CheckSum(void)
{
	CheckSum_Read = false;
	Internal_CheckSum = 0;
}

//ENUM_CMD_STATUS Read_CMD_Status(void)
//{
//	return current_cmd_status;
//}

//void Clear_CMD_Status(void)
//{
//	current_cmd_status = ENUM_CMD_IDLE;
//}

void Next_Repeat_Count_Set(uint8_t new_cnt)
{
	Next_Repeat_Count = new_cnt;
}

uint8_t Next_Repeat_Count_Get(void)
{
	return Next_Repeat_Count;
}

void Next_PWM_Period_Set(uint32_t new_period)
{
	Next_PWM_period = new_period;
}

uint32_t Next_PWM_Period_Get(void)
{
	return Next_PWM_period;
}

void Next_PWM_DutyCycle_Set(uint8_t new_duty_cycle)
{
	Next_PWM_duty_cycle = new_duty_cycle;
}

uint8_t Next_DutyCycle_Period_Get(void)
{
	return Next_PWM_duty_cycle;
}

uint8_t Next_Command_Get(void)
{
    return Next_Command;
}
    
void Next_Command_Clear(void)
{
    Next_Command = ENUM_CMD_IDLE;
}
    
uint32_t Next_Input_Parameter_Get(void)
{
    return Next_input_parameter;
}
    
void ProcessInputChar(uint8_t input_byte)
{
    static uint32_t         		temp_buf = 0;
    ENUM_PARSING_STATE      	    next_state;

    switch (current_state)
    {
     	case ENUM_PARSING_STATE_WAIT_SYNC_BYTE:
     		// Neither 0xff is allowed for repeat count, nor 0xffxx is allowed for pulse-width, nor 0xffffffff is allowed for pulse width, so it is used as "preamble" of data string (at least 4 0xff)
     		if(input_byte!=ENUM_SYNC_BYTE_VALUE)	// Stay in this wait-for-sync state until 0xff occurs
            {
                next_state = ENUM_PARSING_STATE_WAIT_SYNC_BYTE;		// Wait for 0xff --> state unchanged if not 0xff
            }
            else
            {
            	next_state = ENUM_PARSING_STATE_WAIT_COMMAND_CODE;	// go to next state to get a non-0xff next byte
            }
            break;

     	case ENUM_PARSING_STATE_WAIT_COMMAND_CODE:
			if (input_byte == ENUM_SYNC_BYTE_VALUE)					// 0xff still treaded as sync-byte here
			{
				next_state = ENUM_PARSING_STATE_WAIT_COMMAND_CODE;
			}
			else													// Valid data if not 0xff here
			{
				Internal_CheckSum = input_byte;
                Next_Command = input_byte;
                Next_input_parameter = 0;
				CheckSum_Read = false;
                if(input_byte>=CMD_SEND_COMMAND_CODE_ONLY)
                {
                     next_state = ENUM_PARSING_STATE_WAIT_CHECKSUM;
                }
                else if(input_byte>=CMD_SEND_COMMAND_CODE_WITH_BYTE)
                {
                    next_state = ENUM_PARSING_STATE_WAIT_DATA_BYTE;
                }
                else if(input_byte>=CMD_SEND_COMMAND_CODE_WITH_WORD)
                {
                    next_state = ENUM_PARSING_STATE_WAIT_DATA_WORD_HIGH;
                }
                else if (input_byte>=CMD_SEND_COMMAND_CODE_WITH_WORD)
                {
                    next_state = ENUM_PARSING_STATE_WAIT_DATA_DWORD_1ST;
                }
                else if (input_byte==ENUM_CMD_INPUT_TX_SIGNAL)
                {
                    next_state = ENUM_PARSING_STATE_WAIT_REPEAT_COUNT;
                }
                else
                {
                    next_state = ENUM_PARSING_STATE_WAIT_CHECKSUM;
                }
            }
            break;

     	case ENUM_PARSING_STATE_WAIT_REPEAT_COUNT:
                                    
            Next_Repeat_Count_Set(input_byte);
			Internal_CheckSum ^= input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_PWN_DUTY_CYCLE;
            break;

        case ENUM_PARSING_STATE_WAIT_PWN_DUTY_CYCLE:
        	Internal_CheckSum ^= input_byte;
            if(input_byte<=100)
            {
            	Next_PWM_DutyCycle_Set(input_byte);
            	next_state = ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_HIGH;
            }
            else
            {
//				current_cmd_status = ENUM_CMD_REPEAT_COUNT_RECEIVED;		// Repeat-Count received -- and no more
            	next_state = ENUM_PARSING_STATE_WAIT_CHECKSUM;				// a place/chance to signal end-of-packet here if duty-cycle is > 100 - 0xff is recommended
            }
            break;

        case ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_HIGH:
        	Internal_CheckSum ^= input_byte;
            if(input_byte!=0xff)
            {
                temp_buf = input_byte;
                next_state = ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_LOW;
            }
            else
            {
//            	current_cmd_status = ENUM_CMD_REPEAT_COUNT_RECEIVED;
            	next_state = ENUM_PARSING_STATE_WAIT_CHECKSUM;				// a place/chance to signal end-of-packet here if 0xff
            }
            break;

        case ENUM_PARSING_STATE_WAIT_CARRIER_WIDTH_LOW:
        	Internal_CheckSum ^= input_byte;
        	temp_buf = ((temp_buf*256) + input_byte)/8;   // here we use 1us-count, original unit is 1/8us for each count so divided by 8
            if(temp_buf==0)
            {
            	Next_PWM_Period_Set(0xffffffff);
            }
            else
            {
            	Next_PWM_Period_Set(temp_buf);
            }
            IR_data_restart_write_pointer();
            next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_1ST_INPUT;	// Go to wait pulse
            break;

        case ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_1ST_INPUT:
        	Internal_CheckSum ^= input_byte;
            if(input_byte==0xff)
            {
//            	current_cmd_status = ENUM_CMD_WIDTH_DATA_READY;
            	next_state = ENUM_PARSING_STATE_WAIT_CHECKSUM;
            }
            else
            {
            	if((input_byte&0x80)!=0)    // High bit not zero --> 4 bytes width data (highest bit will be removed)
				{
					temp_buf = input_byte;	// Keep highest bit as 1 --> to be removed when last byte received
					next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_2ND;		// 4 byte data
				}
				else // otherwise 2 byte width data
				{
					temp_buf = input_byte;
					next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_WORD_LOW;		// 2 byte data
				}
            }
            break;

        //
        // Finish receiving 2-byte width
        //
        case ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_WORD_LOW:
        	Internal_CheckSum ^= input_byte;
            temp_buf = (temp_buf<<8) + input_byte;
            IR_data_add(temp_buf);
            next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_1ST_INPUT;		// back to wait for 1st byte
            break;
        //END

        //
        //  receiving 4-byte width
        //
        case ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_2ND:
        	Internal_CheckSum ^= input_byte;
       		temp_buf = (temp_buf<<8) + input_byte;
       		next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_3RD;
            break;

        case ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_3RD:
        	Internal_CheckSum ^= input_byte;
            temp_buf = (temp_buf<<8) + input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_4TH;
            break;

        case ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_LONG_4TH:
        	Internal_CheckSum ^= input_byte;
            temp_buf = (temp_buf<<8) + input_byte;
           	temp_buf &= 0x7fffffff; // Remove highest bit here before pushing into queue
            IR_data_add(temp_buf);
            next_state = ENUM_PARSING_STATE_WAIT_PULSE_WIDTH_WAIT_1ST_INPUT;
            break;

        case ENUM_PARSING_STATE_WAIT_CHECKSUM:
            Internal_CheckSum ^= input_byte;
            CheckSum_Read = true;
            next_state = ENUM_PARSING_STATE_WAIT_SYNC_BYTE;
            break;
            //END

        case ENUM_PARSING_STATE_UNKNOWN_STATE:
        case ENUM_PARSING_STATE_UNKNOWN_STATE_INPUT:
            if(input_byte==0xff)
            {
                next_state = ENUM_PARSING_STATE_WAIT_SYNC_BYTE;		// at least 0xffff to be sure
//                current_cmd_status = ENUM_CMD_IDLE;
            }
            else
            {
                next_state = ENUM_PARSING_STATE_UNKNOWN_STATE;
//                current_cmd_status = ENUM_CMD_UNKNOWN;
            }
            break;
//
// Get input parameter
//        
         case ENUM_PARSING_STATE_WAIT_DATA_BYTE:
        	Internal_CheckSum ^= input_byte;
            Next_input_parameter = input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_CHECKSUM;		        // wait for checksum
//            current_cmd_status = ENUM_CMD_INPUT_CMD_RECEIVED;
            break;

        case ENUM_PARSING_STATE_WAIT_DATA_WORD_HIGH:
        	Internal_CheckSum ^= input_byte;
			temp_buf = input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_WORD_LOW; // wait for 2nd byte data
            break;

        case ENUM_PARSING_STATE_WAIT_DATA_WORD_LOW:
        	Internal_CheckSum ^= input_byte;
            Next_input_parameter = (temp_buf<<8) + input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_CHECKSUM;		        // wait for checksum
//            current_cmd_status = ENUM_CMD_INPUT_CMD_RECEIVED;
            break;

         case ENUM_PARSING_STATE_WAIT_DATA_DWORD_1ST:
        	Internal_CheckSum ^= input_byte;
			temp_buf = input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_DWORD_2ND;	        // wait for 2nd byte data
            break;

       case ENUM_PARSING_STATE_WAIT_DATA_DWORD_2ND:
        	Internal_CheckSum ^= input_byte;
       		temp_buf = (temp_buf<<8) + input_byte;
       		next_state = ENUM_PARSING_STATE_WAIT_DATA_DWORD_3RD;            // wait for 3rd byte data
            break;

        case ENUM_PARSING_STATE_WAIT_DATA_DWORD_3RD:
        	Internal_CheckSum ^= input_byte;
            temp_buf = (temp_buf<<8) + input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_DWORD_4TH; // wait for 4th byte data
            break;

        case ENUM_PARSING_STATE_WAIT_DATA_DWORD_4TH:
        	Internal_CheckSum ^= input_byte;
            Next_input_parameter = (temp_buf<<8) + input_byte;
            next_state = ENUM_PARSING_STATE_WAIT_DATA_CHECKSUM;             // wait for checksum   
//            current_cmd_status = ENUM_CMD_INPUT_CMD_RECEIVED;
            break;

        case ENUM_PARSING_STATE_WAIT_DATA_CHECKSUM:
            Internal_CheckSum ^= input_byte;
            CheckSum_Read = true;
            next_state = ENUM_PARSING_STATE_WAIT_SYNC_BYTE; //
            break;
            //END

        default:
            next_state = ENUM_PARSING_STATE_UNKNOWN_STATE;  // Use to catch unknown-unknown situation
//            current_cmd_status = ENUM_CMD_UNKNOWN;
            break;
    }

    current_state = next_state;
}

