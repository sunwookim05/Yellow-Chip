/***************************************************************************//**
* \file mSPI_SPI_UART_PVT.h
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

#if !defined(CY_SCB_SPI_UART_PVT_mSPI_H)
#define CY_SCB_SPI_UART_PVT_mSPI_H

#include "mSPI_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if (mSPI_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  mSPI_rxBufferHead;
    extern volatile uint32  mSPI_rxBufferTail;
    
    /**
    * \addtogroup group_globals
    * @{
    */
    
    /** Sets when internal software receive buffer overflow
     *  was occurred.
    */  
    extern volatile uint8   mSPI_rxBufferOverflow;
    /** @} globals */
#endif /* (mSPI_INTERNAL_RX_SW_BUFFER_CONST) */

#if (mSPI_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  mSPI_txBufferHead;
    extern volatile uint32  mSPI_txBufferTail;
#endif /* (mSPI_INTERNAL_TX_SW_BUFFER_CONST) */

#if (mSPI_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 mSPI_rxBufferInternal[mSPI_INTERNAL_RX_BUFFER_SIZE];
#endif /* (mSPI_INTERNAL_RX_SW_BUFFER) */

#if (mSPI_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 mSPI_txBufferInternal[mSPI_TX_BUFFER_SIZE];
#endif /* (mSPI_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

void mSPI_SpiPostEnable(void);
void mSPI_SpiStop(void);

#if (mSPI_SCB_MODE_SPI_CONST_CFG)
    void mSPI_SpiInit(void);
#endif /* (mSPI_SCB_MODE_SPI_CONST_CFG) */

#if (mSPI_SPI_WAKE_ENABLE_CONST)
    void mSPI_SpiSaveConfig(void);
    void mSPI_SpiRestoreConfig(void);
#endif /* (mSPI_SPI_WAKE_ENABLE_CONST) */

void mSPI_UartPostEnable(void);
void mSPI_UartStop(void);

#if (mSPI_SCB_MODE_UART_CONST_CFG)
    void mSPI_UartInit(void);
#endif /* (mSPI_SCB_MODE_UART_CONST_CFG) */

#if (mSPI_UART_WAKE_ENABLE_CONST)
    void mSPI_UartSaveConfig(void);
    void mSPI_UartRestoreConfig(void);
#endif /* (mSPI_UART_WAKE_ENABLE_CONST) */


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in mSPI_SetPins() */
#define mSPI_UART_RX_PIN_ENABLE    (mSPI_UART_RX)
#define mSPI_UART_TX_PIN_ENABLE    (mSPI_UART_TX)

/* UART RTS and CTS position to be used in  mSPI_SetPins() */
#define mSPI_UART_RTS_PIN_ENABLE    (0x10u)
#define mSPI_UART_CTS_PIN_ENABLE    (0x20u)


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Interrupt processing */
#define mSPI_SpiUartEnableIntRx(intSourceMask)  mSPI_SetRxInterruptMode(intSourceMask)
#define mSPI_SpiUartEnableIntTx(intSourceMask)  mSPI_SetTxInterruptMode(intSourceMask)
uint32  mSPI_SpiUartDisableIntRx(void);
uint32  mSPI_SpiUartDisableIntTx(void);


#endif /* (CY_SCB_SPI_UART_PVT_mSPI_H) */


/* [] END OF FILE */
