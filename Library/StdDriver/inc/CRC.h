/**************************************************************************//**
 * @file     crc.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/09/24 2:52p $
 * @brief    N575 CRC Driver Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup N575_Device_Driver N575 Device Driver
  @{
*/

/** @addtogroup N575_CRC_Driver CRC Driver
  @{
*/

/** @addtogroup N575_CRC_EXPORTED_TYPEDEF CRC Exported Type Defines
  @{
*/

#define	CRC_MSB (0)
#define	CRC_LSB (CRC_CTL_MODE_Msk)

/*@}*/ /* end of group N575_CRC_EXPORTED_CONSTANTS */

/** @addtogroup N575_CRC_EXPORTED_FUNCTIONS CRC Exported Functions
  @{
*/
	
/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t CRC_Open(void);
int32_t CRC_Init(uint32_t eLSB, int32_t i32PacketLen);
int16_t CRC_Calc( uint32_t *Data, int32_t i32PacketLen);
void CRC_Close(void);

/*@}*/ /* end of group N575_CRC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group N575_CRC_Driver */

/*@}*/ /* end of group N575_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
