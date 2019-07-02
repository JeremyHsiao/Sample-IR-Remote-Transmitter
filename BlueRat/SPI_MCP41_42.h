/****************************************************************************
 *   $Id:: timer_app.h 3635 2010-06-02 00:31:46Z usb00423                     $
 *   Project: IR Tx
 *
 *   Description:
 *     This file contains definition and prototype for 32-bit timer 
 *     configuration.
 *
 ****************************************************************************/
#ifndef _SPI_MCP41_42_H_
#define _SPI_MCP41_42_H_

extern void spiWriteRegAddrOnly(const unsigned char regAddr);
extern void spiWriteRegByte(const unsigned char regAddr, const unsigned char regData);
extern void spiWriteRegWord(const unsigned char regAddr, const unsigned short regData);

#define SPI_CS_Port			PB
#define SPI_CS_Pin			0
#define SPI_CS_Bitmask		(~(1<<(SPI_CS_Pin)))		// to-be-updated
#define SPI_CK_Port			PB
#define SPI_CK_Pin			1
#define SPI_CK_Bitmask		(~(1<<(SPI_CK_Pin)))		// to-be-updated
#define SPI_MOSI_Port		PB
#define SPI_MOSI_Pin		7
#define SPI_MOSI_Bitmask	(~(1<<(SPI_MOSI_Pin)))		// to-be-updated
//#define SPI_MISO_Port		PA
//#define SPI_MISO_Pin		3
//#define SPI_MISO_Bitmask	(~(1<<(SPI_MISO_Pin)))		// to-be-updated

#endif /* end _SPI_MCP41_42_H_ */
/*****************************************************************************
**                            End Of File
******************************************************************************/
