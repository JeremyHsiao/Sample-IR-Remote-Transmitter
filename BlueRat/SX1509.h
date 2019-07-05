/****************************************************************************
 *   $Id:: sx1509.h 3635 2010-06-02 00:31:46Z usb00423                     $
 *   Project: Blue
 *
 *   Description:
 *     This file contains definition and prototype for 32-bit timer 
 *     configuration.
 *
 ****************************************************************************/
#ifndef _SX1509_H_
#define _SX1509_H_

extern void SX1509_WriteLowByte(uint16_t output_data);
extern void SX1509_WriteHighByte(uint16_t output_data);
extern void SX1509_WritePin(uint8_t bit_no, uint8_t output_data);
extern void SX1509_TogglePin(uint8_t bit_no);	    

#endif /* end _SX1509_H_ */
/*****************************************************************************
**                            End Of File
******************************************************************************/
