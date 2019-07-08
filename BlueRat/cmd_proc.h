/*
 * cmd_proc.h
 *
 *  Created on: 2017年10月26日
 *      Author: Jeremy.Hsiao
 */

#ifndef _CMD_PROC_H_
#define _CMD_PROC_H_

// V02
typedef enum {
    ENUM_CMD_IDLE = 0,
	ENUM_CMD_UNKNOWN_LAST = 0x7e,
    ENUM_CMD_INPUT_TX_SIGNAL = 0x7f,
    ENUM_CMD_ADD_REPEAT_COUNT = 0x80,
    ENUM_CMD_SET_INPUT_GPIO_DEBOUNCE_TIME_PB1 = 0x81,
    ENUM_CMD_SET_INPUT_GPIO_DEBOUNCE_TIME_PB7 = 0x82,
    ENUM_CMD_CODE_0X83 = 0x83,
    ENUM_CMD_CODE_0X84 = 0x84,
    ENUM_CMD_CODE_0X85 = 0x85,
    ENUM_CMD_CODE_0X86 = 0x86,
    ENUM_CMD_CODE_0X87 = 0x87,
    ENUM_CMD_CODE_0X88 = 0x88,
    ENUM_CMD_CODE_0X89 = 0x89,
    ENUM_CMD_CODE_0X8A = 0x8a,
    ENUM_CMD_CODE_0X8B = 0x8b,
    ENUM_CMD_CODE_0X8C = 0x8c,
    ENUM_CMD_CODE_0X8D = 0x8d,
    ENUM_CMD_CODE_0X8E = 0x8e,
    ENUM_CMD_CODE_0X8F = 0x8f,
    ENUM_CMD_SPI_WRITE_3_BYTE_MODE_00 = 0x90,
    ENUM_CMD_I2C_WRITE_SLAVEADR_WORD = 0x91,
    ENUM_CMD_I2C_READ_N_BYTE = 0x92,
    ENUM_CMD_CODE_0X93 = 0x93,
    ENUM_CMD_CODE_0X94 = 0x94,
    ENUM_CMD_CODE_0X95 = 0x95,
    ENUM_CMD_CODE_0X96 = 0x96,
    ENUM_CMD_CODE_0X97 = 0x97,
    ENUM_CMD_CODE_0X98 = 0x98,
    ENUM_CMD_CODE_0X99 = 0x99,
    ENUM_CMD_CODE_0X9A = 0x9a,
    ENUM_CMD_CODE_0X9B = 0x9b,
    ENUM_CMD_CODE_0X9C = 0x9c,
    ENUM_CMD_CODE_0X9D = 0x9d,
    ENUM_CMD_FORCE_RESTART = 0x9e,
    ENUM_CMD_ENTER_ISP_MODE = 0x9f,
    ENUM_CMD_SET_GPIO_SINGLE_BIT = 0xa0,    // End of command with 2-byte parameter
    ENUM_CMD_CODE_0XA1 = 0xa1,
    ENUM_CMD_CODE_0XA2 = 0xa2,
    ENUM_CMD_CODE_0XA3 = 0xa3,
    ENUM_CMD_CODE_0XA4 = 0xa4,
    ENUM_CMD_CODE_0XA5 = 0xa5,
    ENUM_CMD_CODE_0XA6 = 0xa6,
    ENUM_CMD_CODE_0XA7 = 0xa7,
    ENUM_CMD_CODE_0XA8 = 0xa8,
    ENUM_CMD_CODE_0XA9 = 0xa9,
    ENUM_CMD_CODE_0XAA = 0xaa,
    ENUM_CMD_CODE_0XAB = 0xab,
    ENUM_CMD_CODE_0XAC = 0xac,
    ENUM_CMD_CODE_0XAD = 0xad,
    ENUM_CMD_CODE_0XAE = 0xae,
    ENUM_CMD_CODE_0XAF = 0xaf,
    ENUM_CMD_SPI_WRITE_WORD_MODE_00 = 0xb0,
    ENUM_CMD_I2C_WRITE_SLAVEADR_BYTE = 0xb1,
    ENUM_CMD_SX1509_LOWBYTE_SET = 0xb2,
    ENUM_CMD_SX1509_HIGHBYTE_SET = 0xb3,
    ENUM_CMD_SX1509_WRITE_BIT = 0xb4,
    ENUM_CMD_CODE_0XB5 = 0xb5,
    ENUM_CMD_CODE_0XB6 = 0xb6,
    ENUM_CMD_CODE_0XB7 = 0xb7,
    ENUM_CMD_CODE_0XB8 = 0xb8,
    ENUM_CMD_CODE_0XB9 = 0xb9,
    ENUM_CMD_CODE_0XBA = 0xba,
    ENUM_CMD_CODE_0XBB = 0xbb,
    ENUM_CMD_CODE_0XBC = 0xbc,
    ENUM_CMD_CODE_0XBD = 0xbd,
    ENUM_CMD_CODE_0XBE = 0xbe,
    ENUM_CMD_CODE_0XBF = 0xbf,
    ENUM_CMD_SET_GPIO_ALL_BIT = 0xc0,       // End of command with byte parameter
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
    ENUM_CMD_SPI_WRITE_BYTE_MODE_00 = 0xd0,
    ENUM_CMD_SPI_ENABLE_PB_PORT = 0xd1,
    ENUM_CMD_CODE_0XD2 = 0xd2,
    ENUM_CMD_CODE_0XD3 = 0xd3,
    ENUM_CMD_CODE_0XD4 = 0xd4,
    ENUM_CMD_SX1509_TOGGLE_BIT = 0xd5,
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
    ENUM_CMD_GET_GPIO_INPUT = 0xe0,         // End of command only code
    ENUM_CMD_GET_SENSOR_VALUE = 0xe1,
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
    ENUM_CMD_DETECT_SX1509 = 0xf0,
    ENUM_CMD_READ_SX1509 = 0xf1,
    ENUM_CMD_CODE_0XF2 = 0xf2,
    ENUM_CMD_CODE_0XF3 = 0xf3,
    ENUM_CMD_CODE_0XF4 = 0xf4,
    ENUM_CMD_CODE_0XF5 = 0xf5,
    ENUM_CMD_CODE_0XF6 = 0xf6,
    ENUM_CMD_CODE_0XF7 = 0xf7,
    ENUM_CMD_GET_TX_CURRENT_REPEAT_COUNT = 0xf8,
    ENUM_CMD_GET_TX_RUNNING_STATUS = 0xf9,
    ENUM_CMD_RETURN_SW_VER = 0xfa,
    ENUM_CMD_RETURN_BUILD_TIME = 0xfb,
    ENUM_CMD_RETURN_CMD_VERSION = 0xfc,
    ENUM_CMD_SAY_HI  = 0xfd,
    ENUM_CMD_STOP_ALL    = 0xfe,
    ENUM_SYNC_BYTE_VALUE = 0xff,
    ENUM_CMD_VERSION_V100 = 0x100,
    ENUM_CMD_VERSION_V200 = 0x200,
    ENUM_CMD_VERSION_V201 = 0x201,
    ENUM_CMD_VERSION_V202 = 0x202,
    ENUM_CMD_VERSION_V203 = 0x203,
    ENUM_CMD_VERSION_V204 = 0x204,
    ENUM_CMD_VERSION_CURRENT_PLUS_1,
    ENUM_CMD_STATE_MAX
} ENUM_CMD_STATUS;

#define CMD_CODE_LOWER_LIMIT                        (0x80)
#define CMD_SEND_COMMAND_CODE_WITH_DOUBLE_WORD      (0x80)
#define CMD_SEND_COMMAND_CODE_WITH_WORD             (0xa0)
#define CMD_SEND_COMMAND_CODE_WITH_BYTE             (0xc0)
#define CMD_SEND_COMMAND_CODE_ONLY                  (0xe0)

#define _CMD_SAY_HI_RETURN_HEADER_                        "HI"  
#define _CMD_RETURN_SW_VER_RETURN_HEADER_                 "SW:"  
#define _CMD_BUILD_TIME_RETURN_HEADER_                    "AT:"
#define _CMD_RETURN_CMD_VERSION_RETURN_HEADER_            "CMD_VER:" 
#define _CMD_GET_TX_RUNNING_STATUS_HEADER_                "TX:"  
#define _CMD_GET_TX_CURRENT_REPEAT_COUNT_RETURN_HEADER_   "CNT:"  
#define _CMD_GPIO_INPUT_RETURN_HEADER_                    "IN:"
#define _CMD_SENSOR_INPUT_RETURN_HEADER_                  "SS:"

#define _CMD_IO_EXTEND_DETECT_RETURN_HEADER_              "IO:"
#define _CMD_IO_EXTEND_INPUT_RETURN_HEADER_               "EI:"

extern void CheckIfISP(void);
extern void ProcessInputCommand(void);

extern uint8_t compare_result;

#endif /* !_CMD_PROC_H_ */
