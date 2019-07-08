/*
 * SPI_MCP41_42.c
 *
 *  Created on: Jun 27, 2019
 *      Author: Jeremy.Hsiao
 */

#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "SX1509.h"						 
#include "SPI_MCP41_42.h"

// It is defined within SX1509.h"					
#ifdef SPI_BY_SX1509

void SPI_CS(uint8_t value)
{
	SX1509_WritePin(SPI_CS_SX1509_GPIO,value);
}

void SPI_MOSI(uint8_t value)
{
	SX1509_WritePin(SPI_SI_SX1509_GPIO,value);
}


void SPI_CK(uint8_t value)
{
	SX1509_WritePin(SPI_SCK_SX1509_GPIO,value);
}

#else

void SPI_CS(uint8_t value)
{
	SPI_CS_Port->DATMSK = SPI_CS_Bitmask;
	SPI_CS_Port->DOUT = (value)?0xffffffff:0;
}

void SPI_MOSI(uint8_t value)
{
	SPI_MOSI_Port->DATMSK = SPI_MOSI_Bitmask;
	SPI_MOSI_Port->DOUT = (value)?0xffffffff:0;
}

/*
void SPI_MISO(uint8_t value)
{
	SPI_MISO_Port->DATMSK = SPI_MISO_Bitmask;
	SPI_MISO_Port->DOUT = (value)?0xffffffff:0;
}
 */

void SPI_CK(uint8_t value)
{
	SPI_CK_Port->DATMSK = SPI_CK_Bitmask;
	SPI_CK_Port->DOUT = (value)?0xffffffff:0;
}

#endif // SPI_BY_SX1509

/**************************************************************************************
 * spiWriteRegAddrOnly
 *
 * Writes to an 8-bit register with the SPI port
 **************************************************************************************/
void spiWriteRegAddrOnly(const unsigned char regAddr)
{

  unsigned char SPICount;                                       // Counter used to clock out the data

  unsigned char SPIData;                                        // Define a data structure for the SPI data

  SPI_CS(1);                                        		    // Make sure we start with active-low CS high
  SPI_CK(0);                                        		    // and CK low

  SPIData = regAddr;                                            // Preload the data to be sent with Address
  SPI_CS(0);                                                    // Set active-low CS low to start the SPI cycle
                                                                // Although SPIData could be implemented as an "int",
                                                                // resulting in one
                                                                // loop, the routines run faster when two loops
                                                                // are implemented with
                                                                // SPIData implemented as two "char"s.

  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                         // Check for a 1
      SPI_MOSI(1);                                              // and set the MOSI line appropriately
    else
      SPI_MOSI(0);
    SPI_CK(1);                                                  // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;                                              // Rotate to get the next bit
  }                                                             // and loop back to send the next bit

  SPI_CS(1);
  SPI_MOSI(0);
}

/**************************************************************************************
 * spiWriteRegByte
 *
 * Writes to an 8-bit register with the SPI port
 **************************************************************************************/
void spiWriteRegByte(const unsigned char regAddr, const unsigned char regData)
{

  unsigned char SPICount;                                       // Counter used to clock out the data

  unsigned char SPIData;                                        // Define a data structure for the SPI data

  SPI_CS(1);                                        		    // Make sure we start with active-low CS high
  SPI_CK(0);                                        		    // and CK low

  SPIData = regAddr;                                            // Preload the data to be sent with Address
  SPI_CS(0);                                                    // Set active-low CS low to start the SPI cycle
                                                                // Although SPIData could be implemented as an "int",
                                                                // resulting in one
                                                                // loop, the routines run faster when two loops
                                                                // are implemented with
                                                                // SPIData implemented as two "char"s.

  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                         // Check for a 1
      SPI_MOSI(1);                                              // and set the MOSI line appropriately
    else
      SPI_MOSI(0);
    SPI_CK(1);                                                  // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;                                              // Rotate to get the next bit
  }                                                             // and loop back to send the next bit

                                                                // Repeat for the Data byte
  SPIData = regData;                                            // Preload the data to be sent with Data
  for (SPICount = 0; SPICount < 8; SPICount++)
  {
    if (SPIData & 0x80)
        SPI_MOSI(1);                                             // and set the MOSI line appropriately
    else
        SPI_MOSI(0);
    SPI_CK(1);                                                   // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;
  }
  SPI_CS(1);
  SPI_MOSI(0);
}

/**************************************************************************************
 * spiWriteRegWord
 *
 * Writes to an 8-bit register with the SPI port
 **************************************************************************************/
void spiWriteRegWord(const unsigned char regAddr, const unsigned short regData)
{

  unsigned char SPICount;                                       // Counter used to clock out the data

  unsigned char SPIData;                                        // Define a data structure for the SPI data

  SPI_CS(1);                                        		    // Make sure we start with active-low CS high
  SPI_CK(0);                                        		    // and CK low

  SPIData = regAddr;                                            // Preload the data to be sent with Address
  SPI_CS(0);                                                    // Set active-low CS low to start the SPI cycle
                                                                // Although SPIData could be implemented as an "int",
                                                                // resulting in one
                                                                // loop, the routines run faster when two loops
                                                                // are implemented with
                                                                // SPIData implemented as two "char"s.

  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                         // Check for a 1
      SPI_MOSI(1);                                              // and set the MOSI line appropriately
    else
      SPI_MOSI(0);
    SPI_CK(1);                                                  // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;                                              // Rotate to get the next bit
  }                                                             // and loop back to send the next bit

                                                                // Repeat for the Data byte
  SPIData = (regData>>8)&0xff;                                  // Preload the data to be sent with Data
  for (SPICount = 0; SPICount < 8; SPICount++)
  {
    if (SPIData & 0x80)
        SPI_MOSI(1);                                             // and set the MOSI line appropriately
    else
        SPI_MOSI(0);
    SPI_CK(1);                                                   // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;
  }

  SPIData = regData&0xff;                                  			// Preload the data to be sent with Data
  for (SPICount = 0; SPICount < 8; SPICount++)
  {
    if (SPIData & 0x80)
        SPI_MOSI(1);                                             // and set the MOSI line appropriately
    else
        SPI_MOSI(0);
    SPI_CK(1);                                                   // Toggle the clock line
    SPI_CK(0);
    SPIData <<= 1;
  }

  SPI_CS(1);
  SPI_MOSI(0);
}



/**************************************************************************************
 * spiReadReg
 *
 * Reads an 8-bit register with the SPI port.
 * Data is returned. 
 **************************************************************************************/
/*
unsigned char spiReadReg (const unsigned char regAddr)
{

  unsigned char SPICount;                                       // Counter used to clock out the data
  
  unsigned char SPIData;                  
  
  SPI_CS = 1;                                                   // Make sure we start with active-low CS high
  SPI_CK = 0;                                                   // and CK low
  SPIData = regAddr;                                            // Preload the data to be sent with Address and Data

  SPI_CS = 0;                                                   // Set active-low CS low to start the SPI cycle
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock out the Address and Data
  {
    if (SPIData & 0x80)
      SPI_MOSI = 1;
    else
      SPI_MOSI = 0;
    SPI_CK = 1;
    SPI_CK = 0;
    SPIData <<= 1;
  }                                                             // and loop back to send the next bit
  SPI_MOSI = 0;                                                 // Reset the MOSI data line
  
  SPIData = 0;
  for (SPICount = 0; SPICount < 8; SPICount++)                  // Prepare to clock in the data to be read
  {
    SPIData <<=1;                                               // Rotate the data
    SPI_CK = 1;                                                 // Raise the clock to clock the data out of the MAX7456
    SPIData += SPI_MISO;                                        // Read the data bit
    SPI_CK = 0;                                                 // Drop the clock ready for the next bit
  }                                                             // and loop back
  SPI_CS = 1;                                                   // Raise CS
                      
  return ((unsigned char)SPIData);                              // Finally return the read data
}
*/
