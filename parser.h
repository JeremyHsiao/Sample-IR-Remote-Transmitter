/*
 * parser.h
 *
 *  Created on: 2017年10月26日
 *      Author: Jeremy.Hsiao
 */

#ifndef _PARSER_H_
#define _PARSER_H_

// For portability from MCUXpresso to here
#define     Bool    uint8_t
#define     false   FALSE   
#define     true    TRUE   

typedef enum {
	ENUM_CMD_IDLE = 0,
	ENUM_CMD_RECEIVING,
	ENUM_CMD_STOP_CMD_RECEIVED,
	ENUM_CMD_INPUT_CMD_RECEIVED,
	ENUM_CMD_REPEAT_COUNT_RECEIVED,
	ENUM_CMD_WIDTH_DATA_READY,
	ENUM_CMD_UNKNOWN,
    ENUM_CMD_ADD_REPEAT_COUNT = 0xc0,
    ENUM_CMD_CODE_0XC1 = 0xc1,
    ENUM_CMD_CODE_0XC2 = 0xc2,
    ENUM_CMD_CODE_0XC3 = 0xc3,
    ENUM_CMD_CODE_0XC4 = 0xc4,
    ENUM_CMD_CODE_0XC5 = 0xc5,
    ENUM_CMD_CODE_0XC6 = 0xc6,
    ENUM_CMD_CODE_0XC7 = 0xc7,
    ENUM_CMD_CODE_0XC8 = 0xc8,
    ENUM_CMD_CODE_0XC9 = 0xc9,
    ENUM_CMD_CODE_0XCA = 0xca,
    ENUM_CMD_CODE_0XCB = 0xcb,
    ENUM_CMD_CODE_0XCC = 0xcc,
    ENUM_CMD_CODE_0XCD = 0xcd,
    ENUM_CMD_CODE_0XCE = 0xce,
    ENUM_CMD_CODE_0XCF = 0xcf,
    ENUM_CMD_SET_GPIO_SINGLE_BIT = 0xd0,
    ENUM_CMD_CODE_0XD1 = 0xd1,
    ENUM_CMD_CODE_0XD2 = 0xd2,
    ENUM_CMD_CODE_0XD3 = 0xd3,
    ENUM_CMD_CODE_0XD4 = 0xd4,
    ENUM_CMD_CODE_0XD5 = 0xd5,
    ENUM_CMD_CODE_0XD6 = 0xd6,
    ENUM_CMD_CODE_0XD7 = 0xd7,
    ENUM_CMD_CODE_0XD8 = 0xd8,
    ENUM_CMD_CODE_0XD9 = 0xd9,
    ENUM_CMD_CODE_0XDA = 0xda,
    ENUM_CMD_CODE_0XDB = 0xdb,
    ENUM_CMD_CODE_0XDC = 0xdc,
    ENUM_CMD_CODE_0XDD = 0xdd,
    ENUM_CMD_CODE_0XDE = 0xde,
    ENUM_CMD_CODE_0XDF = 0xdf,
    ENUM_CMD_SET_GPIO_ALL_BIT = 0xe0,
    ENUM_CMD_CODE_0XE1 = 0xe1,
    ENUM_CMD_CODE_0XE2 = 0xe2,
    ENUM_CMD_CODE_0XE3 = 0xe3,
    ENUM_CMD_CODE_0XE4 = 0xe4,
    ENUM_CMD_CODE_0XE5 = 0xe5,
    ENUM_CMD_CODE_0XE6 = 0xe6,
    ENUM_CMD_CODE_0XE7 = 0xe7,
    ENUM_CMD_CODE_0XE8 = 0xe8,
    ENUM_CMD_CODE_0XE9 = 0xe9,
    ENUM_CMD_CODE_0XEA = 0xea,
    ENUM_CMD_CODE_0XEB = 0xeb,
    ENUM_CMD_CODE_0XEC = 0xec,
    ENUM_CMD_CODE_0XED = 0xed,
    ENUM_CMD_CODE_0XEE = 0xee,
    ENUM_CMD_CODE_0XEF = 0xef,
    ENUM_CMD_GET_GPIO_INPUT = 0xf0,
    ENUM_CMD_GET_SENSOR_VALUE = 0xf1,
    ENUM_CMD_CODE_0XF2 = 0xf2,
    ENUM_CMD_CODE_0XF3 = 0xf3,
    ENUM_CMD_CODE_0XF4 = 0xf4,
    ENUM_CMD_CODE_0XF5 = 0xf5,
    ENUM_CMD_CODE_0XF6 = 0xf6,
    ENUM_CMD_CODE_0XF7 = 0xf7,
    ENUM_CMD_CODE_0XF8 = 0xf8,
    ENUM_CMD_CODE_0XF9 = 0xf9,
    ENUM_CMD_CODE_0XFA = 0xfa,
    ENUM_CMD_CODE_0XFB = 0xfb,
    ENUM_CMD_CODE_0XFC = 0xfc,
    ENUM_CMD_CODE_0XFD = 0xfd,
    ENUM_CMD_STOP_ALL = 0xfe,
    ENUM_SYNC_BYTE_VALUE = 0xff,
    ENUM_CMD_STATE_MAX
} ENUM_CMD_STATUS;

#define CMD_SEND_COMMAND_CODE_WITH_DOUBLE_WORD      (0xc0)
#define CMD_SEND_COMMAND_CODE_WITH_WORD             (0xd0)
#define CMD_SEND_COMMAND_CODE_WITH_BYTE             (0xe0)
#define CMD_SEND_COMMAND_CODE_ONLY                  (0xf0)


extern void Init_Parser(void);
extern void ProcessInputChar(uint8_t input_byte);

extern Bool CheckSum_Ready(void);
extern uint8_t Read_CheckSum(void);
extern void Reset_CheckSum(void);
extern ENUM_CMD_STATUS Read_CMD_Status(void);
extern void Clear_CMD_Status(void);
extern void Next_Repeat_Count_Set(uint8_t new_cnt);
extern uint8_t Next_Repeat_Count_Get(void);
extern void Next_PWM_Period_Set(uint32_t new_period);
extern uint32_t Next_PWM_Period_Get(void);
extern void Next_PWM_DutyCycle_Set(uint8_t new_duty_cycle);
extern uint8_t Next_DutyCycle_Period_Get(void);
extern uint8_t Next_Command_Get(void);
extern uint32_t Next_Input_Parameter_Get(void);

#endif /* !_PARSER_H_ */
