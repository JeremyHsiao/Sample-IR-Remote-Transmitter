/**************************************************************************//**
 * @file     i2c_example_main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    N575 I2C Driver Sample Code
 *           This is a I2C master mode demo and need to be tested with a slave device.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "N575.h"
#include "i2c.h"
#include "nuvoton_i2c.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[16];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8TxLen;
volatile uint8_t g_u8RxLen;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8SuccessFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } else {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
//        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
		I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI); // STOP and end
		//g_u8EndFlag = 1;
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
       	if (g_u8DataLen<g_u8TxLen)
		{ 
	    	I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        	I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		}
		else
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
		}
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr | 0x01));  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
		g_u8DataLen = 0;
		if(g_u8RxLen>0)
		{
          	I2C_SET_CONTROL_REG(I2C0, I2C_AA | I2C_SI);
		}
		else
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		}
    } else if (u32Status == 0x50) {             /* DATA has been received and ACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
		g_au8TxData[g_u8DataLen++] =  g_u8RxData;
		if(g_u8DataLen<g_u8RxLen)
		{
          	I2C_SET_CONTROL_REG(I2C0, I2C_AA | I2C_SI);
		}
		else
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		}
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
		g_au8TxData[g_u8DataLen++] =  g_u8RxData;
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
		g_u8SuccessFlag = 1;
        g_u8EndFlag = 1;
    } else {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
		g_u8EndFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                     						   */
/*  Device address + data[0]_as_Register_Address + data                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                      /* START has been transmitted */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
       	if (g_u8TxLen>0)
		{ 
	    	I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        	I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		}
		else
		{
			// Stop if only slave address & no data
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8SuccessFlag = 1;
            g_u8EndFlag = 1;
		}
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        //I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
		I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI); // STOP and end
		g_u8EndFlag = 1;
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen < g_u8TxLen) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
		    g_u8SuccessFlag = 1;
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else {
        /* TO DO */
		I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI); // STOP and end
		g_u8EndFlag = 1;
        //printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

// g_u8DataLen =
//int32_t Write_I2C(uint8_t slvaddr)
//{
//        g_u8EndFlag = 0;
//		g_u8SuccessFlag = 0;
//
//        /* I2C function to write data to slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
//
//        /* I2C as master sends START signal */
//        I2C_SET_CONTROL_REG(I2C0, I2C_STA);
//
//        /* Wait I2C Tx Finish */
//        while (g_u8EndFlag == 0);
//        g_u8EndFlag = 0;
//
//		return 0;
//}

//int32_t Read_I2C(uint8_t slvaddr)
//{
//        g_u8EndFlag = 0;
//		g_u8SuccessFlag = 0;
//
//        /* I2C function to read data from slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
//
//        g_u8DataLen = 0;
//        g_u8DeviceAddr = slvaddr;
//
//        I2C_SET_CONTROL_REG(I2C0, I2C_STA);
//
//        /* Wait I2C Rx Finish */
//        while (g_u8EndFlag == 0);
//
//		return 0;
//}
void init_I2C_Write_global_variable(uint8_t slvaddr)
{
		g_u8DeviceAddr = slvaddr;
		g_u8DataLen = 0;
        g_u8EndFlag = 0;
		g_u8SuccessFlag = 0;
}
   
uint8_t I2C_Write_Byte(uint8_t slvaddr, uint8_t i2c_data)
{
		// Fill data
    	g_au8TxData[0] = (uint8_t)(i2c_data);
    	g_u8TxLen = 1;
		init_I2C_Write_global_variable(slvaddr);

        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA);

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        g_u8EndFlag = 0;

		return g_u8SuccessFlag;
}

uint8_t I2C_Write_Word(uint8_t slvaddr, uint16_t i2c_data)
{
		// Fill data
		g_au8TxData[0] = (uint8_t)((i2c_data & 0xFF00) >> 8);
    	g_au8TxData[1] = (uint8_t)(i2c_data & 0x00FF);
    	g_u8TxLen = 2;
		init_I2C_Write_global_variable(slvaddr);

        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA);

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        g_u8EndFlag = 0;

		return g_u8SuccessFlag;
}

//int32_t Read_Write_SLAVE(uint8_t slvaddr)
//{
//    uint32_t i;
//
//    g_u8DeviceAddr = slvaddr;
//
//    for (i = 0; i < 0x100; i++) {
//        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
//        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
//        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);
//
//        g_u8DataLen = 0;
//        g_u8EndFlag = 0;
//
//        /* I2C function to write data to slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
//
//        /* I2C as master sends START signal */
//        I2C_SET_CONTROL_REG(I2C0, I2C_STA);
//
//        /* Wait I2C Tx Finish */
//        while (g_u8EndFlag == 0);
//        g_u8EndFlag = 0;
//
//        /* I2C function to read data from slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
//
//        g_u8DataLen = 0;
//        g_u8DeviceAddr = slvaddr;
//
//        I2C_SET_CONTROL_REG(I2C0, I2C_STA);
//
//        /* Wait I2C Rx Finish */
//        while (g_u8EndFlag == 0);
//
//        /* Compare data */
//        if (g_u8RxData != g_au8TxData[2]) {
//            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
//            return -1;
//        }
//    }
//    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
//    return 0;
//}
