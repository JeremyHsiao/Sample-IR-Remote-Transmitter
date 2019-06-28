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

#endif /* end _SPI_MCP41_42_H_ */
/*****************************************************************************
**                            End Of File
******************************************************************************/
