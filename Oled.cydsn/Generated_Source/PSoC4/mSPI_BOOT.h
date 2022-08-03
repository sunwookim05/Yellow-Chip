/***************************************************************************//**
* \file mSPI_BOOT.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values of the bootloader
*  communication APIs for the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2014-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_mSPI_H)
#define CY_SCB_BOOT_mSPI_H

#include "mSPI_PVT.h"

#if (mSPI_SCB_MODE_I2C_INC)
    #include "mSPI_I2C.h"
#endif /* (mSPI_SCB_MODE_I2C_INC) */

#if (mSPI_SCB_MODE_EZI2C_INC)
    #include "mSPI_EZI2C.h"
#endif /* (mSPI_SCB_MODE_EZI2C_INC) */

#if (mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC)
    #include "mSPI_SPI_UART.h"
#endif /* (mSPI_SCB_MODE_SPI_INC || mSPI_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define mSPI_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mSPI) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (mSPI_SCB_MODE_I2C_INC)
    #define mSPI_I2C_BTLDR_COMM_ENABLED     (mSPI_BTLDR_COMM_ENABLED && \
                                                            (mSPI_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mSPI_I2C_SLAVE_CONST))
#else
     #define mSPI_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (mSPI_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (mSPI_SCB_MODE_EZI2C_INC)
    #define mSPI_EZI2C_BTLDR_COMM_ENABLED   (mSPI_BTLDR_COMM_ENABLED && \
                                                         mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define mSPI_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (mSPI_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (mSPI_SCB_MODE_SPI_INC)
    #define mSPI_SPI_BTLDR_COMM_ENABLED     (mSPI_BTLDR_COMM_ENABLED && \
                                                            (mSPI_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mSPI_SPI_SLAVE_CONST))
#else
        #define mSPI_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (mSPI_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (mSPI_SCB_MODE_UART_INC)
       #define mSPI_UART_BTLDR_COMM_ENABLED    (mSPI_BTLDR_COMM_ENABLED && \
                                                            (mSPI_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (mSPI_UART_RX_DIRECTION && \
                                                              mSPI_UART_TX_DIRECTION)))
#else
     #define mSPI_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (mSPI_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define mSPI_BTLDR_COMM_MODE_ENABLED    (mSPI_I2C_BTLDR_COMM_ENABLED   || \
                                                     mSPI_SPI_BTLDR_COMM_ENABLED   || \
                                                     mSPI_EZI2C_BTLDR_COMM_ENABLED || \
                                                     mSPI_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void mSPI_I2CCyBtldrCommStart(void);
    void mSPI_I2CCyBtldrCommStop (void);
    void mSPI_I2CCyBtldrCommReset(void);
    cystatus mSPI_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mSPI_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (mSPI_SCB_MODE_I2C_CONST_CFG)
        #define mSPI_CyBtldrCommStart   mSPI_I2CCyBtldrCommStart
        #define mSPI_CyBtldrCommStop    mSPI_I2CCyBtldrCommStop
        #define mSPI_CyBtldrCommReset   mSPI_I2CCyBtldrCommReset
        #define mSPI_CyBtldrCommRead    mSPI_I2CCyBtldrCommRead
        #define mSPI_CyBtldrCommWrite   mSPI_I2CCyBtldrCommWrite
    #endif /* (mSPI_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void mSPI_EzI2CCyBtldrCommStart(void);
    void mSPI_EzI2CCyBtldrCommStop (void);
    void mSPI_EzI2CCyBtldrCommReset(void);
    cystatus mSPI_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mSPI_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (mSPI_SCB_MODE_EZI2C_CONST_CFG)
        #define mSPI_CyBtldrCommStart   mSPI_EzI2CCyBtldrCommStart
        #define mSPI_CyBtldrCommStop    mSPI_EzI2CCyBtldrCommStop
        #define mSPI_CyBtldrCommReset   mSPI_EzI2CCyBtldrCommReset
        #define mSPI_CyBtldrCommRead    mSPI_EzI2CCyBtldrCommRead
        #define mSPI_CyBtldrCommWrite   mSPI_EzI2CCyBtldrCommWrite
    #endif /* (mSPI_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void mSPI_SpiCyBtldrCommStart(void);
    void mSPI_SpiCyBtldrCommStop (void);
    void mSPI_SpiCyBtldrCommReset(void);
    cystatus mSPI_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mSPI_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (mSPI_SCB_MODE_SPI_CONST_CFG)
        #define mSPI_CyBtldrCommStart   mSPI_SpiCyBtldrCommStart
        #define mSPI_CyBtldrCommStop    mSPI_SpiCyBtldrCommStop
        #define mSPI_CyBtldrCommReset   mSPI_SpiCyBtldrCommReset
        #define mSPI_CyBtldrCommRead    mSPI_SpiCyBtldrCommRead
        #define mSPI_CyBtldrCommWrite   mSPI_SpiCyBtldrCommWrite
    #endif /* (mSPI_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void mSPI_UartCyBtldrCommStart(void);
    void mSPI_UartCyBtldrCommStop (void);
    void mSPI_UartCyBtldrCommReset(void);
    cystatus mSPI_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mSPI_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (mSPI_SCB_MODE_UART_CONST_CFG)
        #define mSPI_CyBtldrCommStart   mSPI_UartCyBtldrCommStart
        #define mSPI_CyBtldrCommStop    mSPI_UartCyBtldrCommStop
        #define mSPI_CyBtldrCommReset   mSPI_UartCyBtldrCommReset
        #define mSPI_CyBtldrCommRead    mSPI_UartCyBtldrCommRead
        #define mSPI_CyBtldrCommWrite   mSPI_UartCyBtldrCommWrite
    #endif /* (mSPI_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_BTLDR_COMM_ENABLED)
    #if (mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void mSPI_CyBtldrCommStart(void);
        void mSPI_CyBtldrCommStop (void);
        void mSPI_CyBtldrCommReset(void);
        cystatus mSPI_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus mSPI_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mSPI)
        #define CyBtldrCommStart    mSPI_CyBtldrCommStart
        #define CyBtldrCommStop     mSPI_CyBtldrCommStop
        #define CyBtldrCommReset    mSPI_CyBtldrCommReset
        #define CyBtldrCommWrite    mSPI_CyBtldrCommWrite
        #define CyBtldrCommRead     mSPI_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mSPI) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mSPI_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define mSPI_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define mSPI_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define mSPI_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define mSPI_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef mSPI_SPI_BYTE_TO_BYTE
    #define mSPI_SPI_BYTE_TO_BYTE   (11u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef mSPI_UART_BYTE_TO_BYTE
    #define mSPI_UART_BYTE_TO_BYTE  (2500u)
#endif /* mSPI_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_mSPI_H) */


/* [] END OF FILE */
