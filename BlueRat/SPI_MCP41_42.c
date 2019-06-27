/*
 * SPI_MCP41_42.c
 *
 *  Created on: Jun 27, 2019
 *      Author: Jeremy.Hsiao
 */

#include <stdio.h>
#include <string.h>
#include "N575.h"
#include "SPI_MCP41_42.h"

#define SPI_CS_Port			PA
#define SPI_CS_Bitmask		(~(1<<10))		// to-be-updated
#define SPI_MOSI_Port		PA
#define SPI_MOSI_Bitmask	(~(1<<11))		// to-be-updated
#define SPI_MISO_Port		PA
#define SPI_MISO_Bitmask	(~(1<<12))		// to-be-updated
#define SPI_CK_Port			PA
#define SPI_CK_Bitmask		(~(1<<13))		// to-be-updated

static inline void BIT_SET(sfr,bitmask) //
{
  // mask value 0 means to write/clear
  PA->DATMSK = bitmask;
  PA->DOUT = 0xffffffff;
}

static inline void BIT_CLR(sfr,bitmask) //
{
  // mask value 0 means to write/clear
  PA->DATMSK = bitmask;
  PA->DOUT = 0;
}

void SPI_CS(uint8_t value)
{
	(value)?BIT_SET(SPI_CS_Port,SPI_CS_Bitmask):BIT_CLR(SPI_CS_Port,SPI_CS_Bitmask);
}

void SPI_MOSI(uint8_t value)
{
	(value)?BIT_SET(SPI_MOSI_Port,SPI_MOSI_Bitmask):BIT_CLR(SPI_MOSI_Port,SPI_MOSI_Bitmask);
}

void SPI_MISO(uint8_t value)
{
	(value)?BIT_SET(SPI_MISO_Port,SPI_MISO_Bitmask):BIT_CLR(SPI_MISO_Port,SPI_MISO_Bitmask);
}

void SPI_CK(uint8_t value)
{
	(value)?BIT_SET(SPI_CK_Port,SPI_CK_Bitmask):BIT_CLR(SPI_CK_Port,SPI_CK_Bitmask);
}


/**************************************************************************************
 * spiWriteReg
 *
 * Writes to an 8-bit register with the SPI port
 **************************************************************************************/
void spiWriteReg(const unsigned char regAddr, const unsigned char regData)
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