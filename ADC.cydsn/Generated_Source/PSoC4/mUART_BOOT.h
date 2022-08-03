/***************************************************************************//**
* \file mUART_BOOT.h
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

#if !defined(CY_SCB_BOOT_mUART_H)
#define CY_SCB_BOOT_mUART_H

#include "mUART_PVT.h"

#if (mUART_SCB_MODE_I2C_INC)
    #include "mUART_I2C.h"
#endif /* (mUART_SCB_MODE_I2C_INC) */

#if (mUART_SCB_MODE_EZI2C_INC)
    #include "mUART_EZI2C.h"
#endif /* (mUART_SCB_MODE_EZI2C_INC) */

#if (mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC)
    #include "mUART_SPI_UART.h"
#endif /* (mUART_SCB_MODE_SPI_INC || mUART_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define mUART_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (mUART_SCB_MODE_I2C_INC)
    #define mUART_I2C_BTLDR_COMM_ENABLED     (mUART_BTLDR_COMM_ENABLED && \
                                                            (mUART_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mUART_I2C_SLAVE_CONST))
#else
     #define mUART_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (mUART_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (mUART_SCB_MODE_EZI2C_INC)
    #define mUART_EZI2C_BTLDR_COMM_ENABLED   (mUART_BTLDR_COMM_ENABLED && \
                                                         mUART_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define mUART_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (mUART_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (mUART_SCB_MODE_SPI_INC)
    #define mUART_SPI_BTLDR_COMM_ENABLED     (mUART_BTLDR_COMM_ENABLED && \
                                                            (mUART_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             mUART_SPI_SLAVE_CONST))
#else
        #define mUART_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (mUART_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (mUART_SCB_MODE_UART_INC)
       #define mUART_UART_BTLDR_COMM_ENABLED    (mUART_BTLDR_COMM_ENABLED && \
                                                            (mUART_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (mUART_UART_RX_DIRECTION && \
                                                              mUART_UART_TX_DIRECTION)))
#else
     #define mUART_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (mUART_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define mUART_BTLDR_COMM_MODE_ENABLED    (mUART_I2C_BTLDR_COMM_ENABLED   || \
                                                     mUART_SPI_BTLDR_COMM_ENABLED   || \
                                                     mUART_EZI2C_BTLDR_COMM_ENABLED || \
                                                     mUART_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void mUART_I2CCyBtldrCommStart(void);
    void mUART_I2CCyBtldrCommStop (void);
    void mUART_I2CCyBtldrCommReset(void);
    cystatus mUART_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mUART_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (mUART_SCB_MODE_I2C_CONST_CFG)
        #define mUART_CyBtldrCommStart   mUART_I2CCyBtldrCommStart
        #define mUART_CyBtldrCommStop    mUART_I2CCyBtldrCommStop
        #define mUART_CyBtldrCommReset   mUART_I2CCyBtldrCommReset
        #define mUART_CyBtldrCommRead    mUART_I2CCyBtldrCommRead
        #define mUART_CyBtldrCommWrite   mUART_I2CCyBtldrCommWrite
    #endif /* (mUART_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void mUART_EzI2CCyBtldrCommStart(void);
    void mUART_EzI2CCyBtldrCommStop (void);
    void mUART_EzI2CCyBtldrCommReset(void);
    cystatus mUART_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mUART_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (mUART_SCB_MODE_EZI2C_CONST_CFG)
        #define mUART_CyBtldrCommStart   mUART_EzI2CCyBtldrCommStart
        #define mUART_CyBtldrCommStop    mUART_EzI2CCyBtldrCommStop
        #define mUART_CyBtldrCommReset   mUART_EzI2CCyBtldrCommReset
        #define mUART_CyBtldrCommRead    mUART_EzI2CCyBtldrCommRead
        #define mUART_CyBtldrCommWrite   mUART_EzI2CCyBtldrCommWrite
    #endif /* (mUART_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void mUART_SpiCyBtldrCommStart(void);
    void mUART_SpiCyBtldrCommStop (void);
    void mUART_SpiCyBtldrCommReset(void);
    cystatus mUART_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mUART_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (mUART_SCB_MODE_SPI_CONST_CFG)
        #define mUART_CyBtldrCommStart   mUART_SpiCyBtldrCommStart
        #define mUART_CyBtldrCommStop    mUART_SpiCyBtldrCommStop
        #define mUART_CyBtldrCommReset   mUART_SpiCyBtldrCommReset
        #define mUART_CyBtldrCommRead    mUART_SpiCyBtldrCommRead
        #define mUART_CyBtldrCommWrite   mUART_SpiCyBtldrCommWrite
    #endif /* (mUART_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void mUART_UartCyBtldrCommStart(void);
    void mUART_UartCyBtldrCommStop (void);
    void mUART_UartCyBtldrCommReset(void);
    cystatus mUART_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus mUART_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (mUART_SCB_MODE_UART_CONST_CFG)
        #define mUART_CyBtldrCommStart   mUART_UartCyBtldrCommStart
        #define mUART_CyBtldrCommStop    mUART_UartCyBtldrCommStop
        #define mUART_CyBtldrCommReset   mUART_UartCyBtldrCommReset
        #define mUART_CyBtldrCommRead    mUART_UartCyBtldrCommRead
        #define mUART_CyBtldrCommWrite   mUART_UartCyBtldrCommWrite
    #endif /* (mUART_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_BTLDR_COMM_ENABLED)
    #if (mUART_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void mUART_CyBtldrCommStart(void);
        void mUART_CyBtldrCommStop (void);
        void mUART_CyBtldrCommReset(void);
        cystatus mUART_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus mUART_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART)
        #define CyBtldrCommStart    mUART_CyBtldrCommStart
        #define CyBtldrCommStop     mUART_CyBtldrCommStop
        #define CyBtldrCommReset    mUART_CyBtldrCommReset
        #define CyBtldrCommWrite    mUART_CyBtldrCommWrite
        #define CyBtldrCommRead     mUART_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (mUART_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define mUART_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define mUART_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define mUART_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define mUART_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef mUART_SPI_BYTE_TO_BYTE
    #define mUART_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef mUART_UART_BYTE_TO_BYTE
    #define mUART_UART_BYTE_TO_BYTE  (171u)
#endif /* mUART_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_mUART_H) */


/* [] END OF FILE */
