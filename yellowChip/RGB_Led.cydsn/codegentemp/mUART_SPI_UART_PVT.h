/***************************************************************************//**
* \file mUART_SPI_UART_PVT.h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_PVT_mUART_H)
#define CY_SCB_SPI_UART_PVT_mUART_H

#include "mUART_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if (mUART_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  mUART_rxBufferHead;
    extern volatile uint32  mUART_rxBufferTail;
    
    /**
    * \addtogroup group_globals
    * @{
    */
    
    /** Sets when internal software receive buffer overflow
     *  was occurred.
    */  
    extern volatile uint8   mUART_rxBufferOverflow;
    /** @} globals */
#endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

#if (mUART_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  mUART_txBufferHead;
    extern volatile uint32  mUART_txBufferTail;
#endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */

#if (mUART_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 mUART_rxBufferInternal[mUART_INTERNAL_RX_BUFFER_SIZE];
#endif /* (mUART_INTERNAL_RX_SW_BUFFER) */

#if (mUART_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 mUART_txBufferInternal[mUART_TX_BUFFER_SIZE];
#endif /* (mUART_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

void mUART_SpiPostEnable(void);
void mUART_SpiStop(void);

#if (mUART_SCB_MODE_SPI_CONST_CFG)
    void mUART_SpiInit(void);
#endif /* (mUART_SCB_MODE_SPI_CONST_CFG) */

#if (mUART_SPI_WAKE_ENABLE_CONST)
    void mUART_SpiSaveConfig(void);
    void mUART_SpiRestoreConfig(void);
#endif /* (mUART_SPI_WAKE_ENABLE_CONST) */

void mUART_UartPostEnable(void);
void mUART_UartStop(void);

#if (mUART_SCB_MODE_UART_CONST_CFG)
    void mUART_UartInit(void);
#endif /* (mUART_SCB_MODE_UART_CONST_CFG) */

#if (mUART_UART_WAKE_ENABLE_CONST)
    void mUART_UartSaveConfig(void);
    void mUART_UartRestoreConfig(void);
#endif /* (mUART_UART_WAKE_ENABLE_CONST) */


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in mUART_SetPins() */
#define mUART_UART_RX_PIN_ENABLE    (mUART_UART_RX)
#define mUART_UART_TX_PIN_ENABLE    (mUART_UART_TX)

/* UART RTS and CTS position to be used in  mUART_SetPins() */
#define mUART_UART_RTS_PIN_ENABLE    (0x10u)
#define mUART_UART_CTS_PIN_ENABLE    (0x20u)


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Interrupt processing */
#define mUART_SpiUartEnableIntRx(intSourceMask)  mUART_SetRxInterruptMode(intSourceMask)
#define mUART_SpiUartEnableIntTx(intSourceMask)  mUART_SetTxInterruptMode(intSourceMask)
uint32  mUART_SpiUartDisableIntRx(void);
uint32  mUART_SpiUartDisableIntTx(void);


#endif /* (CY_SCB_SPI_UART_PVT_mUART_H) */


/* [] END OF FILE */
