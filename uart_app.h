/**************************************************************************//**
 * @file     uart_app.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/11/23 11:55p $
 * @brief    Uart code - non-driver
 *
 * @note
 *
 *
 ******************************************************************************/
#ifndef _UART_APP_H_
#define _UART_APP_H_

extern void UART0_IRQHandler(void);
extern void UART_init(void);
extern int itoa_10(uint32_t value, char* result);

#endif // !_UART_APP_H_
