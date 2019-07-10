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

void SX1509_Init_SPI_Pin(void)
{
	 SX1509_WritePin_UnMasked(SPI_CS_SX1509_GPIO, 1);
	 SX1509_WritePin_UnMasked(SPI_SCK_SX1509_GPIO, 0);
}

uint32_t Process_Value_Mask_for_SPI_PIN(uint32_t input_value)
{
	uint32_t	SPI_value, Other_value;

   	SPI_value = IO_Extend_Output_Value & MASK_FOR_SPI_ON_IO_EXPENDER; 	// Keep current SPI pin value
	Other_value = input_value & (~MASK_FOR_SPI_ON_IO_EXPENDER); 		// Filter out SPI pin value for new input.
	return (SPI_value|Other_value);  
}

void SX1509_WriteLowWord(uint16_t output_data)
{
	const uint8_t	low_slave_adr = (0x3e<<1);
	uint32_t		Temp_Value;
	
	Temp_Value = output_data;
	Temp_Value |= IO_Extend_Output_Value & 0xffff0000;
	IO_Extend_Output_Value = Process_Value_Mask_for_SPI_PIN(Temp_Value);
	Temp_Value = IO_Extend_Output_Value&0xffff;

	I2C_Write_Word(low_slave_adr, (uint16_t)(0x0e00));
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x0f00));
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x1000|((Temp_Value>>8)&0xff)) );
	I2C_Write_Word(low_slave_adr, (uint16_t)(0x1100|((Temp_Value)&0xff)) );
}

void SX1509_WriteHighWord(uint16_t output_data)
{
	const uint8_t	high_slave_adr = (0x70<<1);
	uint32_t		Temp_Value;

	Temp_Value = output_data;
	Temp_Value <<= 16;
	Temp_Value |= IO_Extend_Output_Value & 0x0000ffff;
	IO_Extend_Output_Value = Process_Value_Mask_for_SPI_PIN(Temp_Value);
	Temp_Value = IO_Extend_Output_Value>>16;

	I2C_Write_Word(high_slave_adr, (uint16_t)(0x0e00));
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x0f00));
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x1000|((Temp_Value>>8)&0xff)) );
	I2C_Write_Word(high_slave_adr, (uint16_t)(0x1100|((Temp_Value)&0xff)) );
}

void SX1509_WritePin_UnMasked(uint8_t bit_no, uint8_t output_data)
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

void SX1509_WritePin(uint8_t bit_no, uint8_t output_data)
{
	if(bit_no==SPI_SCK_SX1509_GPIO)
	{
		return;
	}
	if(bit_no==SPI_CS_SX1509_GPIO)
	{
		return;
	}
	if(bit_no==SPI_SI_SX1509_GPIO)
	{
		return;
	}
	SX1509_WritePin_UnMasked(bit_no, output_data);
}

void SX1509_TogglePin(uint8_t bit_no)
{
	// IO_Extend_Output_Value will be updated inside SX1509_WritePin()
	SX1509_WritePin(bit_no, (IO_Extend_Output_Value&(1UL<<bit_no))?0:1);		// 1->0 or 0->1
}
