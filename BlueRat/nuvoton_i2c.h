/*
 *  nuvoton_i2c.h
 *  Created on: 2017年10月26日
 *      Author: Jeremy.Hsiao
 */

#ifndef _NUVOTON_I2C_H_
#define _NUVOTON_I2C_H_

// For portability from MCUXpresso to here
//extern void I2C_MasterRx(uint32_t u32Status) ;
//extern void I2C_MasterTx(uint32_t u32Status)  ;
//extern int32_t Read_Write_SLAVE(uint8_t slvaddr) ;

extern uint8_t I2C_Write_SlaveAdr_Only(uint8_t slvaddr);
extern uint8_t I2C_Write_Byte(uint8_t slvaddr, uint8_t  i2c_data);
extern uint8_t I2C_Write_Word(uint8_t slvaddr, uint16_t i2c_data);
extern uint8_t I2C_Write_3Byte(uint8_t slvaddr, uint32_t i2c_data);
extern uint8_t I2C_Write_Reg_with_Long(uint8_t slvaddr, uint8_t regaddr, uint32_t i2c_data);
extern uint8_t I2C_Read_N_Byte_from_RegAddr(uint8_t slvaddr, uint8_t regaddr, uint8_t n_byte);
extern uint8_t GetDataIndex(uint8_t index);

extern uint8_t g_au8TxData[16];

#endif /* !_NUVOTON_I2C_H_ */
