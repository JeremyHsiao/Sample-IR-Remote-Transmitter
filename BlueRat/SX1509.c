/*
 * SX1509.c
 *
 *  Created on: Jun 27, 2019
 *      Author: Jeremy.Hsiao
 */

#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "nuvoton_i2c.h"
#include "SX1509.h"

uint32_t	IO_Extend_Output_Value=0;

void SX1509_WriteLowByte(uint16_t output_data)
{
	const uint8_t	low_slave_adr = (0x3e<<1);
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x0e00));
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x0f00));
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x1000|((output_data>>8)&0xff)) );
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x1100|((output_data)&0xff)) );
	IO_Extend_Output_Value = (IO_Extend_Output_Value & 0xffff0000) | (output_data&0xffff);
}

void SX1509_WriteHighByte(uint16_t output_data)
{
	const uint8_t	high_slave_adr = (0x70<<1);
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x0e00));
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x0f00));
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x1000|((output_data>>8)&0xff)) );
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x1100|((output_data)&0xff)) );
	IO_Extend_Output_Value = (IO_Extend_Output_Value & 0x0000ffff) | ((output_data<<16)&0xffff0000);
}

void SX1509_WritePin(uint8_t bit_no, uint8_t output_data)
{
	if(output_data&0xff)
	{
		IO_Extend_Output_Value |= (1UL<<bit_no);
	}
	else
	{
		IO_Extend_Output_Value &= ~(1UL<<bit_no);
	}
																					  	
	if(bit_no>=24)	  // MSB
	{
		I2C_Write_Word((0x70<<1), (uint16_t)(0x1000|((IO_Extend_Output_Value>>24)&0xff)) );
	}
	else if(bit_no>=16)		// 2nd MSB
	{
		I2C_Write_Word((0x70<<1), (uint16_t)(0x1100|((IO_Extend_Output_Value>>16)&0xff)) );
	}
	else if(bit_no>=8)		// 3rd MSB
	{
		I2C_Write_Word((0x3e<<1), (uint16_t)(0x1000|((IO_Extend_Output_Value>>8)&0xff)) );
	}
	else					// LSB
	{
		I2C_Write_Word((0x3e<<1), (uint16_t)(0x1100|((IO_Extend_Output_Value)&0xff)) );
	}
}

void SX1509_TogglePin(uint8_t bit_no)
{
	// IO_Extend_Output_Value will be updated inside SX1509_WritePin()
	SX1509_WritePin(bit_no, (IO_Extend_Output_Value&(1UL<<bit_no))?0:1);		// 1->0 or 0->1
}
