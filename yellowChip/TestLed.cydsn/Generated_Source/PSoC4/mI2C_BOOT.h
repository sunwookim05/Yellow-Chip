/***************************************************************************//**
* \file mI2C_BOOT.h
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

#if !defined(CY_SCB_BOOT_mI2C_H)
#define CY_SCB_BOOT_mI2C_H

#include "mI2C_PVT.h"

#if (mI2C_SCB_MODE_I2C_INC)
    #include "mI2C_I2C.h"
#endif /* (mI2C_SCB_MODE_I2C_INC) */

#if (mI2C_SCB_MODE_EZI2C_INC)
    #include "mI2C_EZI2C.h"
#endif /* (mI2C_SCB_MODE_EZI2C_INC) */

#if (mI2C_SCB_MODE_SPI_INC || mI2C_SCB_MODE_UART_INC)
    #include "mI2C_SPI_UART.h"
#endif /* (mI2C_SCB_MODE_SPI_INC || mI2C_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define mI2C_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mI2C) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (mI2C_SCB_MODE_I2C_INC)
    #define mI2C_I2C_BTLDR_COMM_ENABLED     (mI2C_BTLDR_COMM_ENABLED && \
                                                            (mI2C_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mI2C_I2C_SLAVE_CONST))
#else
     #define mI2C_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (mI2C_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (mI2C_SCB_MODE_EZI2C_INC)
    #define mI2C_EZI2C_BTLDR_COMM_ENABLED   (mI2C_BTLDR_COMM_ENABLED && \
                                                         mI2C_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define mI2C_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (mI2C_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (mI2C_SCB_MODE_SPI_INC)
    #define mI2C_SPI_BTLDR_COMM_ENABLED     (mI2C_BTLDR_COMM_ENABLED && \
                                                            (mI2C_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mI2C_SPI_SLAVE_CONST))
#else
        #define mI2C_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (mI2C_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (mI2C_SCB_MODE_UART_INC)
       #define mI2C_UART_BTLDR_COMM_ENABLED    (mI2C_BTLDR_COMM_ENABLED && \
                                                            (mI2C_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (mI2C_UART_RX_DIRECTION && \
                                                              mI2C_UART_TX_DIRECTION)))
#else
     #define mI2C_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (mI2C_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define mI2C_BTLDR_COMM_MODE_ENABLED    (mI2C_I2C_BTLDR_COMM_ENABLED   || \
                                                     mI2C_SPI_BTLDR_COMM_ENABLED   || \
                                                     mI2C_EZI2C_BTLDR_COMM_ENABLED || \
                                                     mI2C_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void mI2C_I2CCyBtldrCommStart(void);
    void mI2C_I2CCyBtldrCommStop (void);
    void mI2C_I2CCyBtldrCommReset(void);
    cystatus mI2C_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mI2C_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (mI2C_SCB_MODE_I2C_CONST_CFG)
        #define mI2C_CyBtldrCommStart   mI2C_I2CCyBtldrCommStart
        #define mI2C_CyBtldrCommStop    mI2C_I2CCyBtldrCommStop
        #define mI2C_CyBtldrCommReset   mI2C_I2CCyBtldrCommReset
        #define mI2C_CyBtldrCommRead    mI2C_I2CCyBtldrCommRead
        #define mI2C_CyBtldrCommWrite   mI2C_I2CCyBtldrCommWrite
    #endif /* (mI2C_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void mI2C_EzI2CCyBtldrCommStart(void);
    void mI2C_EzI2CCyBtldrCommStop (void);
    void mI2C_EzI2CCyBtldrCommReset(void);
    cystatus mI2C_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mI2C_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (mI2C_SCB_MODE_EZI2C_CONST_CFG)
        #define mI2C_CyBtldrCommStart   mI2C_EzI2CCyBtldrCommStart
        #define mI2C_CyBtldrCommStop    mI2C_EzI2CCyBtldrCommStop
        #define mI2C_CyBtldrCommReset   mI2C_EzI2CCyBtldrCommReset
        #define mI2C_CyBtldrCommRead    mI2C_EzI2CCyBtldrCommRead
        #define mI2C_CyBtldrCommWrite   mI2C_EzI2CCyBtldrCommWrite
    #endif /* (mI2C_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void mI2C_SpiCyBtldrCommStart(void);
    void mI2C_SpiCyBtldrCommStop (void);
    void mI2C_SpiCyBtldrCommReset(void);
    cystatus mI2C_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mI2C_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (mI2C_SCB_MODE_SPI_CONST_CFG)
        #define mI2C_CyBtldrCommStart   mI2C_SpiCyBtldrCommStart
        #define mI2C_CyBtldrCommStop    mI2C_SpiCyBtldrCommStop
        #define mI2C_CyBtldrCommReset   mI2C_SpiCyBtldrCommReset
        #define mI2C_CyBtldrCommRead    mI2C_SpiCyBtldrCommRead
        #define mI2C_CyBtldrCommWrite   mI2C_SpiCyBtldrCommWrite
    #endif /* (mI2C_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void mI2C_UartCyBtldrCommStart(void);
    void mI2C_UartCyBtldrCommStop (void);
    void mI2C_UartCyBtldrCommReset(void);
    cystatus mI2C_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mI2C_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (mI2C_SCB_MODE_UART_CONST_CFG)
        #define mI2C_CyBtldrCommStart   mI2C_UartCyBtldrCommStart
        #define mI2C_CyBtldrCommStop    mI2C_UartCyBtldrCommStop
        #define mI2C_CyBtldrCommReset   mI2C_UartCyBtldrCommReset
        #define mI2C_CyBtldrCommRead    mI2C_UartCyBtldrCommRead
        #define mI2C_CyBtldrCommWrite   mI2C_UartCyBtldrCommWrite
    #endif /* (mI2C_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_BTLDR_COMM_ENABLED)
    #if (mI2C_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void mI2C_CyBtldrCommStart(void);
        void mI2C_CyBtldrCommStop (void);
        void mI2C_CyBtldrCommReset(void);
        cystatus mI2C_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus mI2C_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (mI2C_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mI2C)
        #define CyBtldrCommStart    mI2C_CyBtldrCommStart
        #define CyBtldrCommStop     mI2C_CyBtldrCommStop
        #define CyBtldrCommReset    mI2C_CyBtldrCommReset
        #define CyBtldrCommWrite    mI2C_CyBtldrCommWrite
        #define CyBtldrCommRead     mI2C_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mI2C) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mI2C_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define mI2C_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define mI2C_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define mI2C_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define mI2C_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef mI2C_SPI_BYTE_TO_BYTE
    #define mI2C_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef mI2C_UART_BYTE_TO_BYTE
    #define mI2C_UART_BYTE_TO_BYTE  (2500u)
#endif /* mI2C_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_mI2C_H) */


/* [] END OF FILE */
