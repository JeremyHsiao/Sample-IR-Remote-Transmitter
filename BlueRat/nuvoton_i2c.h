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

extern uint8_t I2C_Write_Byte(uint8_t slvaddr, uint8_t  i2c_data);
extern uint8_t I2C_Write_Word(uint8_t slvaddr, uint16_t i2c_data);

#endif /* !_NUVOTON_I2C_H_ */
