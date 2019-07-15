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

#ifdef SPI_BY_SX1509
void SX1509_Init_SPI_Pin(void)
{
	const uint8_t	low_slave_adr = (0x3e<<1);
	const uint8_t	high_slave_adr = (0x70<<1);

	I2C_Write_3Byte(low_slave_adr,  (uint32_t)(0x0e0000));
	I2C_Write_3Byte(high_slave_adr, (uint32_t)(0x0e0000));
	SX1509_WritePin_UnMasked(SPI_CS_SX1509_GPIO, 1);
	SX1509_WritePin_UnMasked(SPI_SCK_SX1509_GPIO, 0);
}
#endif // #ifdef SPI_BY_SX1509

void SX1509_RESET(void)
{
	SX1509_NRESET_GPIO_PORT->DATMSK = SX1509_NRESET_GPIO_MASK;
	SX1509_NRESET_GPIO_PORT->DOUT = 0;

	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	SX1509_NRESET_GPIO_PORT->DOUT = 0xFFFFFFFF;

#ifdef SPI_BY_SX1509
	SX1509_Init_SPI_Pin();
#endif // #ifdef SPI_BY_SX1509
}

uint32_t Process_Value_Mask_for_SPI_PIN(uint32_t input_value)
{
#ifdef SPI_BY_SX1509
	uint32_t	SPI_value, Other_value;
   	SPI_value = IO_Extend_Output_Value & MASK_FOR_SPI_ON_IO_EXPENDER; 	// Keep current SPI pin value
	Other_value = input_value & (~MASK_FOR_SPI_ON_IO_EXPENDER); 		// Filter out SPI pin value for new input.
	return (SPI_value|Other_value);  
#else
	return input_value;
#endif // #ifdef SPI_BY_SX1509
}

void SX1509_WriteLowWord(uint16_t output_data)
{
	const uint8_t	low_slave_adr = (0x3e<<1);
	uint32_t		Temp_Value;
	
	Temp_Value = output_data;
	Temp_Value |= IO_Extend_Output_Value & 0xffff0000;
	IO_Extend_Output_Value = Process_Value_Mask_for_SPI_PIN(Temp_Value);
	Temp_Value = IO_Extend_Output_Value&0xffff;
	I2C_Write_3Byte(low_slave_adr, 0x100000|Temp_Value);
}

void SX1509_WriteHighWord(uint16_t output_data)
{
	const uint8_t	high_slave_adr = (0x70<<1);
	uint32_t		Temp_Value;

	Temp_Value = output_data;
	Temp_Value <<= 16;
	Temp_Value |= IO_Extend_Output_Value & 0x0000ffff;
	IO_Extend_Output_Value = Process_Value_Mask_for_SPI_PIN(Temp_Value);
	Temp_Value = (IO_Extend_Output_Value>>16)&0xffff;
	I2C_Write_3Byte(high_slave_adr, 0x100000|Temp_Value);
}

void SX1509_WritePin_UnMasked(uint8_t bit_no, uint8_t output_data)
{
	uint8_t		slave_addr;
	uint32_t	i2c_data;

	if(output_data)
	{
		IO_Extend_Output_Value |= (1UL<<bit_no);
	}
	else
	{
		IO_Extend_Output_Value &= ~(1UL<<bit_no);
	}

	if(bit_no>=16)
	{
		i2c_data = ((IO_Extend_Output_Value>>16)&0xffff) | 0x100000;
		slave_addr = (0x70<<1);
	}
	else
	{
		i2c_data = ((IO_Extend_Output_Value    )&0xffff) | 0x100000;
		slave_addr = (0x3e<<1);
	}
	I2C_Write_3Byte(slave_addr, i2c_data);
}

void SX1509_WritePin(uint8_t bit_no, uint8_t output_data)
{
#ifdef SPI_BY_SX1509
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
#endif // #ifdef SPI_BY_SX1509
	SX1509_WritePin_UnMasked(bit_no, output_data);
}

void SX1509_TogglePin(uint8_t bit_no)
{
	// IO_Extend_Output_Value will be updated inside SX1509_WritePin()
	SX1509_WritePin(bit_no, (IO_Extend_Output_Value&(1UL<<bit_no))?0:1);		// 1->0 or 0->1
}
