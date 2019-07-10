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

extern void SX1509_WriteLowWord(uint16_t output_data);
extern void SX1509_WriteHighWord(uint16_t output_data);
extern void SX1509_WritePin(uint8_t bit_no, uint8_t output_data);
extern void SX1509_WritePin_UnMasked(uint8_t bit_no, uint8_t output_data);
extern void SX1509_TogglePin(uint8_t bit_no);	  
extern void SX1509_Init_SPI_Pin(void);
  

#define SPI_BY_SX1509
#ifdef SPI_BY_SX1509
  #define SPI_SCK_SX1509_GPIO				(31-16)
  #define SPI_CS_SX1509_GPIO				(30-16)
  #define SPI_SI_SX1509_GPIO				(29-16)
  #define MASK_FOR_SPI_ON_IO_EXPENDER		((1UL<<SPI_CS_SX1509_GPIO)|(1UL<<SPI_SI_SX1509_GPIO)|(1UL<<SPI_SCK_SX1509_GPIO))		
#endif // SPI_BY_SX1509

#endif /* end _SX1509_H_ */
/*****************************************************************************
**                            End Of File
******************************************************************************/
