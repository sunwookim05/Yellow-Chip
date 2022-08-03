/***************************************************************************//**
* \file mSPI_SPI_UART.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_mSPI_H)
#define CY_SCB_SPI_UART_mSPI_H

#include "mSPI.h"


/***************************************
*   SPI Initial Parameter Constants
****************************************/

#define mSPI_SPI_MODE                   (1u)
#define mSPI_SPI_SUB_MODE               (0u)
#define mSPI_SPI_CLOCK_MODE             (3u)
#define mSPI_SPI_OVS_FACTOR             (16u)
#define mSPI_SPI_MEDIAN_FILTER_ENABLE   (0u)
#define mSPI_SPI_LATE_MISO_SAMPLE_ENABLE (0u)
#define mSPI_SPI_RX_DATA_BITS_NUM       (8u)
#define mSPI_SPI_TX_DATA_BITS_NUM       (8u)
#define mSPI_SPI_WAKE_ENABLE            (0u)
#define mSPI_SPI_BITS_ORDER             (1u)
#define mSPI_SPI_TRANSFER_SEPARATION    (1u)
#define mSPI_SPI_NUMBER_OF_SS_LINES     (1u)
#define mSPI_SPI_RX_BUFFER_SIZE         (8u)
#define mSPI_SPI_TX_BUFFER_SIZE         (8u)

#define mSPI_SPI_INTERRUPT_MODE         (0u)

#define mSPI_SPI_INTR_RX_MASK           (0x0u)
#define mSPI_SPI_INTR_TX_MASK           (0x0u)

#define mSPI_SPI_RX_TRIGGER_LEVEL       (7u)
#define mSPI_SPI_TX_TRIGGER_LEVEL       (0u)

#define mSPI_SPI_BYTE_MODE_ENABLE       (0u)
#define mSPI_SPI_FREE_RUN_SCLK_ENABLE   (0u)
#define mSPI_SPI_SS0_POLARITY           (0u)
#define mSPI_SPI_SS1_POLARITY           (0u)
#define mSPI_SPI_SS2_POLARITY           (0u)
#define mSPI_SPI_SS3_POLARITY           (0u)


/***************************************
*   UART Initial Parameter Constants
****************************************/

#define mSPI_UART_SUB_MODE              (0u)
#define mSPI_UART_DIRECTION             (3u)
#define mSPI_UART_DATA_BITS_NUM         (8u)
#define mSPI_UART_PARITY_TYPE           (2u)
#define mSPI_UART_STOP_BITS_NUM         (2u)
#define mSPI_UART_OVS_FACTOR            (12u)
#define mSPI_UART_IRDA_LOW_POWER        (0u)
#define mSPI_UART_MEDIAN_FILTER_ENABLE  (0u)
#define mSPI_UART_RETRY_ON_NACK         (0u)
#define mSPI_UART_IRDA_POLARITY         (0u)
#define mSPI_UART_DROP_ON_FRAME_ERR     (0u)
#define mSPI_UART_DROP_ON_PARITY_ERR    (0u)
#define mSPI_UART_WAKE_ENABLE           (0u)
#define mSPI_UART_RX_BUFFER_SIZE        (8u)
#define mSPI_UART_TX_BUFFER_SIZE        (8u)
#define mSPI_UART_MP_MODE_ENABLE        (0u)
#define mSPI_UART_MP_ACCEPT_ADDRESS     (0u)
#define mSPI_UART_MP_RX_ADDRESS         (0x2u)
#define mSPI_UART_MP_RX_ADDRESS_MASK    (0xFFu)

#define mSPI_UART_INTERRUPT_MODE        (0u)

#define mSPI_UART_INTR_RX_MASK          (0x0u)
#define mSPI_UART_INTR_TX_MASK          (0x0u)

#define mSPI_UART_RX_TRIGGER_LEVEL      (7u)
#define mSPI_UART_TX_TRIGGER_LEVEL      (0u)

#define mSPI_UART_BYTE_MODE_ENABLE      (0u)
#define mSPI_UART_CTS_ENABLE            (0u)
#define mSPI_UART_CTS_POLARITY          (0u)
#define mSPI_UART_RTS_ENABLE            (0u)
#define mSPI_UART_RTS_POLARITY          (0u)
#define mSPI_UART_RTS_FIFO_LEVEL        (4u)

#define mSPI_UART_RX_BREAK_WIDTH        (11u)

/* SPI mode enum */
#define mSPI_SPI_SLAVE  (0u)
#define mSPI_SPI_MASTER (1u)

/* UART direction enum */
#define mSPI_UART_RX    (1u)
#define mSPI_UART_TX    (2u)
#define mSPI_UART_TX_RX (3u)


/***************************************
*   Conditional Compilation Parameters
****************************************/

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Mode */
    #define mSPI_SPI_SLAVE_CONST        (1u)
    #define mSPI_SPI_MASTER_CONST       (1u)

    /* Direction */
    #define mSPI_RX_DIRECTION           (1u)
    #define mSPI_TX_DIRECTION           (1u)
    #define mSPI_UART_RX_DIRECTION      (1u)
    #define mSPI_UART_TX_DIRECTION      (1u)

    /* Only external RX and TX buffer for Uncofigured mode */
    #define mSPI_INTERNAL_RX_SW_BUFFER   (0u)
    #define mSPI_INTERNAL_TX_SW_BUFFER   (0u)

    /* Get RX and TX buffer size */
    #define mSPI_INTERNAL_RX_BUFFER_SIZE    (mSPI_rxBufferSize + 1u)
    #define mSPI_RX_BUFFER_SIZE             (mSPI_rxBufferSize)
    #define mSPI_TX_BUFFER_SIZE             (mSPI_txBufferSize)

    /* Return true if buffer is provided */
    #define mSPI_CHECK_RX_SW_BUFFER (NULL != mSPI_rxBuffer)
    #define mSPI_CHECK_TX_SW_BUFFER (NULL != mSPI_txBuffer)

    /* Always provide global variables to support RX and TX buffers */
    #define mSPI_INTERNAL_RX_SW_BUFFER_CONST    (1u)
    #define mSPI_INTERNAL_TX_SW_BUFFER_CONST    (1u)

    /* Get wakeup enable option */
    #define mSPI_SPI_WAKE_ENABLE_CONST  (1u)
    #define mSPI_UART_WAKE_ENABLE_CONST (1u)
    #define mSPI_CHECK_SPI_WAKE_ENABLE  ((0u != mSPI_scbEnableWake) && mSPI_SCB_MODE_SPI_RUNTM_CFG)
    #define mSPI_CHECK_UART_WAKE_ENABLE ((0u != mSPI_scbEnableWake) && mSPI_SCB_MODE_UART_RUNTM_CFG)

    /* SPI/UART: TX or RX FIFO size */
    #if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
        #define mSPI_SPI_UART_FIFO_SIZE             (mSPI_FIFO_SIZE)
        #define mSPI_CHECK_UART_RTS_CONTROL_FLOW    (0u)
    #else
        #define mSPI_SPI_UART_FIFO_SIZE (mSPI_GET_FIFO_SIZE(mSPI_CTRL_REG & \
                                                                                    mSPI_CTRL_BYTE_MODE))

        #define mSPI_CHECK_UART_RTS_CONTROL_FLOW \
                    ((mSPI_SCB_MODE_UART_RUNTM_CFG) && \
                     (0u != mSPI_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(mSPI_UART_FLOW_CTRL_REG)))
    #endif /* (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

#else

    /* Internal RX and TX buffer: for SPI or UART */
    #if (mSPI_SCB_MODE_SPI_CONST_CFG)

        /* SPI Direction */
        #define mSPI_SPI_RX_DIRECTION (1u)
        #define mSPI_SPI_TX_DIRECTION (1u)

        /* Get FIFO size */
        #define mSPI_SPI_UART_FIFO_SIZE mSPI_GET_FIFO_SIZE(mSPI_SPI_BYTE_MODE_ENABLE)

        /* SPI internal RX and TX buffers */
        #define mSPI_INTERNAL_SPI_RX_SW_BUFFER  (mSPI_SPI_RX_BUFFER_SIZE > \
                                                                mSPI_SPI_UART_FIFO_SIZE)
        #define mSPI_INTERNAL_SPI_TX_SW_BUFFER  (mSPI_SPI_TX_BUFFER_SIZE > \
                                                                mSPI_SPI_UART_FIFO_SIZE)

        /* Internal SPI RX and TX buffer */
        #define mSPI_INTERNAL_RX_SW_BUFFER  (mSPI_INTERNAL_SPI_RX_SW_BUFFER)
        #define mSPI_INTERNAL_TX_SW_BUFFER  (mSPI_INTERNAL_SPI_TX_SW_BUFFER)

        /* Internal SPI RX and TX buffer size */
        #define mSPI_INTERNAL_RX_BUFFER_SIZE    (mSPI_SPI_RX_BUFFER_SIZE + 1u)
        #define mSPI_RX_BUFFER_SIZE             (mSPI_SPI_RX_BUFFER_SIZE)
        #define mSPI_TX_BUFFER_SIZE             (mSPI_SPI_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define mSPI_SPI_WAKE_ENABLE_CONST  (0u != mSPI_SPI_WAKE_ENABLE)
        #define mSPI_UART_WAKE_ENABLE_CONST (0u)

    #else

        /* UART Direction */
        #define mSPI_UART_RX_DIRECTION (0u != (mSPI_UART_DIRECTION & mSPI_UART_RX))
        #define mSPI_UART_TX_DIRECTION (0u != (mSPI_UART_DIRECTION & mSPI_UART_TX))

        /* Get FIFO size */
        #define mSPI_SPI_UART_FIFO_SIZE mSPI_GET_FIFO_SIZE(mSPI_UART_BYTE_MODE_ENABLE)

        /* UART internal RX and TX buffers */
        #define mSPI_INTERNAL_UART_RX_SW_BUFFER  (mSPI_UART_RX_BUFFER_SIZE > \
                                                                mSPI_SPI_UART_FIFO_SIZE)
        #define mSPI_INTERNAL_UART_TX_SW_BUFFER  (mSPI_UART_TX_BUFFER_SIZE > \
                                                                    mSPI_SPI_UART_FIFO_SIZE)

        /* Internal UART RX and TX buffer */
        #define mSPI_INTERNAL_RX_SW_BUFFER  (mSPI_INTERNAL_UART_RX_SW_BUFFER)
        #define mSPI_INTERNAL_TX_SW_BUFFER  (mSPI_INTERNAL_UART_TX_SW_BUFFER)

        /* Internal UART RX and TX buffer size */
        #define mSPI_INTERNAL_RX_BUFFER_SIZE    (mSPI_UART_RX_BUFFER_SIZE + 1u)
        #define mSPI_RX_BUFFER_SIZE             (mSPI_UART_RX_BUFFER_SIZE)
        #define mSPI_TX_BUFFER_SIZE             (mSPI_UART_TX_BUFFER_SIZE)

        /* Get wakeup enable option */
        #define mSPI_SPI_WAKE_ENABLE_CONST  (0u)
        #define mSPI_UART_WAKE_ENABLE_CONST (0u != mSPI_UART_WAKE_ENABLE)

    #endif /* (mSPI_SCB_MODE_SPI_CONST_CFG) */

    /* Mode */
    #define mSPI_SPI_SLAVE_CONST    (mSPI_SPI_MODE == mSPI_SPI_SLAVE)
    #define mSPI_SPI_MASTER_CONST   (mSPI_SPI_MODE == mSPI_SPI_MASTER)

    /* Direction */
    #define mSPI_RX_DIRECTION ((mSPI_SCB_MODE_SPI_CONST_CFG) ? \
                                            (mSPI_SPI_RX_DIRECTION) : (mSPI_UART_RX_DIRECTION))

    #define mSPI_TX_DIRECTION ((mSPI_SCB_MODE_SPI_CONST_CFG) ? \
                                            (mSPI_SPI_TX_DIRECTION) : (mSPI_UART_TX_DIRECTION))

    /* Internal RX and TX buffer: for SPI or UART. Used in conditional compilation check */
    #define mSPI_CHECK_RX_SW_BUFFER (mSPI_INTERNAL_RX_SW_BUFFER)
    #define mSPI_CHECK_TX_SW_BUFFER (mSPI_INTERNAL_TX_SW_BUFFER)

    /* Provide global variables to support RX and TX buffers */
    #define mSPI_INTERNAL_RX_SW_BUFFER_CONST    (mSPI_INTERNAL_RX_SW_BUFFER)
    #define mSPI_INTERNAL_TX_SW_BUFFER_CONST    (mSPI_INTERNAL_TX_SW_BUFFER)

    /* Wake up enable */
    #define mSPI_CHECK_SPI_WAKE_ENABLE  (mSPI_SPI_WAKE_ENABLE_CONST)
    #define mSPI_CHECK_UART_WAKE_ENABLE (mSPI_UART_WAKE_ENABLE_CONST)

    /* UART flow control: not applicable for CY_SCBIP_V0 || CY_SCBIP_V1 */
    #define mSPI_CHECK_UART_RTS_CONTROL_FLOW    (mSPI_SCB_MODE_UART_CONST_CFG && \
                                                             mSPI_UART_RTS_ENABLE)

#endif /* End (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*       Type Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/

/* mSPI_SPI_INIT_STRUCT */
typedef struct
{
    /** Mode of operation for SPI. The following defines are available choices:
     *  - mSPI_SPI_SLAVE
     *  - mSPI_SPI_MASTE
    */
    uint32 mode;

    /** Submode of operation for SPI. The following defines are available
     *  choices:
     *  - mSPI_SPI_MODE_MOTOROLA
     *  - mSPI_SPI_MODE_TI_COINCIDES
     *  - mSPI_SPI_MODE_TI_PRECEDES
     *  - mSPI_SPI_MODE_NATIONAL
    */
    uint32 submode;

    /** Determines the sclk relationship for Motorola submode. Ignored
     *  for other submodes. The following defines are available choices:
     *  - mSPI_SPI_SCLK_CPHA0_CPOL0
     *  - mSPI_SPI_SCLK_CPHA0_CPOL1
     *  - mSPI_SPI_SCLK_CPHA1_CPOL0
     *  - mSPI_SPI_SCLK_CPHA1_CPOL1
    */
    uint32 sclkMode;

    /** Oversampling factor for the SPI clock. Ignored for Slave mode operation.
    */
    uint32 oversample;

    /** Applies median filter on the input lines: 0 – not applied, 1 – applied.
    */
    uint32 enableMedianFilter;

    /** Applies late sampling of MISO line: 0 – not applied, 1 – applied.
     *  Ignored for slave mode.
    */
    uint32 enableLateSampling;

    /** Enables wakeup from low power mode: 0 – disable, 1 – enable.
     *  Ignored for master mode.
    */
    uint32 enableWake;

    /** Number of data bits for RX direction.
     *  Different dataBitsRx and dataBitsTx are only allowed for National
     *  submode.
    */
    uint32 rxDataBits;

    /** Number of data bits for TX direction.
     *  Different dataBitsRx and dataBitsTx are only allowed for National
     *  submode.
    */
    uint32 txDataBits;

    /** Determines the bit ordering. The following defines are available
     *  choices:
     *  - mSPI_BITS_ORDER_LSB_FIRST
     *  - mSPI_BITS_ORDER_MSB_FIRST
    */
    uint32 bitOrder;

    /** Determines whether transfers are back to back or have SS disabled
     *  between words. Ignored for slave mode. The following defines are
     *  available choices:
     *  - mSPI_SPI_TRANSFER_CONTINUOUS
     *  - mSPI_SPI_TRANSFER_SEPARATED
    */
    uint32 transferSeperation;

    /** Size of the RX buffer in bytes/words (depends on rxDataBits parameter).
     *  A value equal to the RX FIFO depth implies the usage of buffering in
     *  hardware. A value greater than the RX FIFO depth results in a software
     *  buffer.
     *  The mSPI_INTR _RX_NOT_EMPTY interrupt has to be enabled to
     *  transfer data into the software buffer.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words for PSoC 4100 /
     *    PSoC 4200 devices.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words or 16
     *    bytes (Byte mode is enabled) for PSoC 4100 BLE / PSoC 4200 BLE /
     *    PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *    PSoC Analog Coprocessor devices.
    */
    uint32 rxBufferSize;

    /** Buffer space provided for a RX software buffer:
     *  - A NULL pointer must be provided to use hardware buffering.
     *  - A pointer to an allocated buffer must be provided to use software
     *    buffering. The buffer size must equal (rxBufferSize + 1) in bytes if
     *    dataBitsRx is less or equal to 8, otherwise (2 * (rxBufferSize + 1))
     *    in bytes. The software RX buffer always keeps one element empty.
     *    For correct operation the allocated RX buffer has to be one element
     *    greater than maximum packet size expected to be received.
    */
    uint8* rxBuffer;

    /** Size of the TX buffer in bytes/words(depends on txDataBits parameter).
     *  A value equal to the TX FIFO depth implies the usage of buffering in
     *  hardware. A value greater than the TX FIFO depth results in a software
     *  buffer.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words for PSoC 4100 /
     *    PSoC 4200 devices.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words or 16
     *    bytes (Byte mode is enabled) for PSoC 4100 BLE / PSoC 4200 BLE /
     *    PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *    PSoC Analog Coprocessor devices.
    */
    uint32 txBufferSize;

    /** Buffer space provided for a TX software buffer:
     *  - A NULL pointer must be provided to use hardware buffering.
     *  - A pointer to an allocated buffer must be provided to use software
     *    buffering. The buffer size must equal txBufferSize if dataBitsTx is
     *    less or equal to 8, otherwise (2* txBufferSize).
    */
    uint8* txBuffer;

    /** Enables component interrupt: 0 – disable, 1 – enable.
     *  The interrupt has to be enabled if software buffer is used.
    */
    uint32 enableInterrupt;

    /** Mask of enabled interrupt sources for the RX direction. This mask is
     *  written regardless of the setting of the enable Interrupt field.
     *  Multiple sources are enabled by providing a value that is the OR of
     *  all of the following sources to enable:
     *  - mSPI_INTR_RX_FIFO_LEVEL
     *  - mSPI_INTR_RX_NOT_EMPTY
     *  - mSPI_INTR_RX_FULL
     *  - mSPI_INTR_RX_OVERFLOW
     *  - mSPI_INTR_RX_UNDERFLOW
     *  - mSPI_INTR_SLAVE_SPI_BUS_ERROR
    */
    uint32 rxInterruptMask;

    /** FIFO level for an RX FIFO level interrupt. This value is written
     *  regardless of whether the RX FIFO level interrupt source is enabled.
    */
    uint32 rxTriggerLevel;

    /** Mask of enabled interrupt sources for the TX direction. This mask is
     *  written regardless of the setting of the enable Interrupt field.
     *  Multiple sources are enabled by providing a value that is the OR of
     *  all of the following sources to enable:
     *  - mSPI_INTR_TX_FIFO_LEVEL
     *  - mSPI_INTR_TX_NOT_FULL
     *  - mSPI_INTR_TX_EMPTY
     *  - mSPI_INTR_TX_OVERFLOW
     *  - mSPI_INTR_TX_UNDERFLOW
     *  - mSPI_INTR_MASTER_SPI_DONE
    */
    uint32 txInterruptMask;

    /** FIFO level for a TX FIFO level interrupt. This value is written
     * regardless of whether the TX FIFO level interrupt source is enabled.
    */
    uint32 txTriggerLevel;

    /** When enabled the TX and RX FIFO depth is doubled and equal to
     *  16 bytes: 0 – disable, 1 – enable. This implies that number of
     *  TX and RX data bits must be less than or equal to 8.
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 enableByteMode;

    /** Enables continuous SCLK generation by the SPI master: 0 – disable,
     *  1 – enable.
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 enableFreeRunSclk;

    /** Active polarity of slave select lines 0-3. This is bit mask where bit
     *  mSPI_SPI_SLAVE_SELECT0 corresponds to slave select 0
     *  polarity, bit mSPI_SPI_SLAVE_SELECT1 – slave select 1
     *  polarity and so on. Polarity constants are:
     *  - mSPI_SPI_SS_ACTIVE_LOW
     *  - mSPI_SPI_SS_ACTIVE_HIGH
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 polaritySs;
} mSPI_SPI_INIT_STRUCT;


/* mSPI_UART_INIT_STRUCT */
typedef struct
{
    /** Mode of operation for the UART. The following defines are available
     *  choices:
     *  - mSPI_UART_MODE_STD
     *  - mSPI_UART_MODE_SMARTCARD
     *  - mSPI_UART_MODE_IRDA
    */
    uint32 mode;

    /** Direction of operation for the UART. The following defines are available
     *  choices:
     *  - mSPI_UART_TX_RX
     *  - mSPI_UART_RX
     *  - mSPI_UART_TX
    */
    uint32 direction;

    /** Number of data bits.
    */
    uint32 dataBits;

    /** Determines the parity. The following defines are available choices:
     *  - mSPI_UART_PARITY_EVEN
     *  - mSPI_UART_PARITY_ODD
     *  - mSPI_UART_PARITY_NONE
    */
    uint32 parity;

    /** Determines the number of stop bits. The following defines are available
     *  choices:
     *  - mSPI_UART_STOP_BITS_1
     *  - mSPI_UART_STOP_BITS_1_5
     *  - mSPI_UART_STOP_BITS_2
    */
    uint32 stopBits;

    /** Oversampling factor for the UART.
     *
     *  Note The oversampling factor values are changed when enableIrdaLowPower
     *  is enabled:
     *  - mSPI_UART_IRDA_LP_OVS16
     *  - mSPI_UART_IRDA_LP_OVS32
     *  - mSPI_UART_IRDA_LP_OVS48
     *  - mSPI_UART_IRDA_LP_OVS96
     *  - mSPI_UART_IRDA_LP_OVS192
     *  - mSPI_UART_IRDA_LP_OVS768
     *  - mSPI_UART_IRDA_LP_OVS1536
    */
    uint32 oversample;

    /** Enables IrDA low power RX mode operation: 0 – disable, 1 – enable.
     *  The TX functionality does not work when enabled.
    */
    uint32 enableIrdaLowPower;

    /** Applies median filter on the input lines:  0 – not applied, 1 – applied.
    */
    uint32 enableMedianFilter;

    /** Enables retry when NACK response was received: 0 – disable, 1 – enable.
     *  Only current content of TX FIFO is re-sent.
     *  Ignored for modes other than SmartCard.
    */
    uint32 enableRetryNack;

    /** Inverts polarity of RX line: 0 – non-inverting, 1 – inverting.
     *  Ignored for modes other than IrDA.
    */
    uint32 enableInvertedRx;

    /** Drop data from RX FIFO if parity error is detected: 0 – disable,
     *  1 – enable.
    */
    uint32 dropOnParityErr;

    /** Drop data from RX FIFO if a frame error is detected: 0 – disable,
     *  1 – enable.
    */
    uint32 dropOnFrameErr;

    /** Enables wakeup from low power mode: 0 – disable, 1 – enable.
     *  Ignored for modes other than standard UART. The RX functionality
     *  has to be enabled.
    */
    uint32 enableWake;

    /** Size of the RX buffer in bytes/words (depends on rxDataBits parameter).
     *  A value equal to the RX FIFO depth implies the usage of buffering in
     *  hardware. A value greater than the RX FIFO depth results in a software
     *  buffer.
     *  The mSPI_INTR _RX_NOT_EMPTY interrupt has to be enabled to
     *  transfer data into the software buffer.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words for PSoC 4100 /
     *    PSoC 4200 devices.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words or 16
     *    bytes (Byte mode is enabled) for PSoC 4100 BLE / PSoC 4200 BLE /
     *    PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *    PSoC Analog Coprocessor devices.
    */
    uint32 rxBufferSize;

    /** Buffer space provided for a RX software buffer:
     *  - A NULL pointer must be provided to use hardware buffering.
     *  - A pointer to an allocated buffer must be provided to use software
     *    buffering. The buffer size must equal (rxBufferSize + 1) in bytes if
     *    dataBitsRx is less or equal to 8, otherwise (2 * (rxBufferSize + 1))
     *    in bytes. The software RX buffer always keeps one element empty.
     *    For correct operation the allocated RX buffer has to be one element
     *    greater than maximum packet size expected to be received.
    */
    uint8* rxBuffer;

    /** Size of the TX buffer in bytes/words(depends on txDataBits parameter).
     *  A value equal to the TX FIFO depth implies the usage of buffering in
     *  hardware. A value greater than the TX FIFO depth results in a software
     *  buffer.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words for PSoC 4100 /
     *    PSoC 4200 devices.
     *  - The RX and TX FIFO depth is equal to 8 bytes/words or 16
     *    bytes (Byte mode is enabled) for PSoC 4100 BLE / PSoC 4200 BLE /
     *    PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *    PSoC Analog Coprocessor devices.
    */
    uint32 txBufferSize;

    /** Buffer space provided for a TX software buffer:
     *  - A NULL pointer must be provided to use hardware buffering.
     *  - A pointer to an allocated buffer must be provided to use software
     *    buffering. The buffer size must equal txBufferSize if dataBitsTx is
     *    less or equal to 8, otherwise (2* txBufferSize).
    */
    uint8* txBuffer;

    /** Enables multiprocessor mode: 0 – disable, 1 – enable.
    */
    uint32 enableMultiproc;

    /** Enables matched address to be accepted: 0 – disable, 1 – enable.
    */
    uint32 multiprocAcceptAddr;

    /** 8 bit address to match in Multiprocessor mode. Ignored for other modes.
    */
    uint32 multiprocAddr;

    /** 8 bit mask of address bits that are compared for a Multiprocessor
     *  address match. Ignored for other modes.
     *  - Bit value 0 – excludes bit from address comparison.
     *  - Bit value 1 – the bit needs to match with the corresponding bit
     *   of the device address.
    */
    uint32 multiprocAddrMask;

    /** Enables component interrupt: 0 – disable, 1 – enable.
     *  The interrupt has to be enabled if software buffer is used.
    */
    uint32 enableInterrupt;

    /** Mask of interrupt sources to enable in the RX direction. This mask is
     *  written regardless of the setting of the enableInterrupt field.
     *  Multiple sources are enabled by providing a value that is the OR of
     *  all of the following sources to enable:
     *  - mSPI_INTR_RX_FIFO_LEVEL
     *  - mSPI_INTR_RX_NOT_EMPTY
     *  - mSPI_INTR_RX_FULL
     *  - mSPI_INTR_RX_OVERFLOW
     *  - mSPI_INTR_RX_UNDERFLOW
     *  - mSPI_INTR_RX_FRAME_ERROR
     *  - mSPI_INTR_RX_PARITY_ERROR
    */
    uint32 rxInterruptMask;

    /** FIFO level for an RX FIFO level interrupt. This value is written
     *  regardless of whether the RX FIFO level interrupt source is enabled.
    */
    uint32 rxTriggerLevel;

    /** Mask of interrupt sources to enable in the TX direction. This mask is
     *  written regardless of the setting of the enableInterrupt field.
     *  Multiple sources are enabled by providing a value that is the OR of
     *  all of the following sources to enable:
     *  - mSPI_INTR_TX_FIFO_LEVEL
     *  - mSPI_INTR_TX_NOT_FULL
     *  - mSPI_INTR_TX_EMPTY
     *  - mSPI_INTR_TX_OVERFLOW
     *  - mSPI_INTR_TX_UNDERFLOW
     *  - mSPI_INTR_TX_UART_DONE
     *  - mSPI_INTR_TX_UART_NACK
     *  - mSPI_INTR_TX_UART_ARB_LOST
    */
    uint32 txInterruptMask;

    /** FIFO level for a TX FIFO level interrupt. This value is written
     *  regardless of whether the TX FIFO level interrupt source is enabled.
    */
    uint32 txTriggerLevel;

    /** When enabled the TX and RX FIFO depth is doubled and equal to
     *  16 bytes: 0 – disable, 1 – enable. This implies that number of
     *  Data bits must be less than or equal to 8.
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 enableByteMode;

    /** Enables usage of CTS input signal by the UART transmitter : 0 – disable,
     *  1 – enable.
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 enableCts;

    /** Sets active polarity of CTS input signal:
     *  - mSPI_UART_CTS_ACTIVE_LOW
     *  - mSPI_UART_CTS_ACTIVE_HIGH
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 ctsPolarity;

    /** RX FIFO level for RTS signal activation. While the RX FIFO has fewer
     *  entries than the RTS FIFO level value the RTS signal remains active,
     *  otherwise the RTS signal becomes inactive. By setting this field to 0,
     *  RTS signal activation is disabled.
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 rtsRxFifoLevel;

    /** Sets active polarity of RTS output signal:
     *  - mSPI_UART_RTS_ ACTIVE_LOW
     *  - mSPI_UART_RTS_ACTIVE_HIGH
     *
     *  Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     *  PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     *  PSoC Analog Coprocessor.
    */
    uint8 rtsPolarity;

    /** Configures the width of a break signal in that triggers the break
     *  detection interrupt source. A Break is a low level on the RX line.
     *  Valid range is 1-16 UART bits times.
    */
    uint8 breakWidth;
} mSPI_UART_INIT_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes
***************************************/

/**
* \addtogroup group_spi
* @{
*/
/* SPI specific functions */
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    void mSPI_SpiInit(const mSPI_SPI_INIT_STRUCT *config);
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */


#if(mSPI_SCB_MODE_SPI_INC)
    /*******************************************************************************
    * Function Name: mSPI_SpiIsBusBusy
    ****************************************************************************//**
    *
    *  Returns the current status on the bus. The bus status is determined using
    *  the slave select signal.
    *  - Motorola and National Semiconductor sub-modes: The bus is busy after
    *    the slave select line is activated and lasts until the slave select line
    *    is deactivated.
    *  - Texas Instrument sub-modes: The bus is busy at the moment of the initial
    *    pulse on the slave select line and lasts until the transfer is complete.
    *    If SPI Master is configured to use "Separated transfers"
    *    (see Continuous versus Separated Transfer Separation), the bus is busy
    *    during each element transfer and is free between each element transfer.
    *    The Master does not activate SS line immediately after data has been
    *    written into the TX FIFO.
    *
    *  \return slaveSelect: Current status on the bus.
    *   If the returned value is nonzero, the bus is busy.
    *   If zero is returned, the bus is free. The bus status is determined using
    *   the slave select signal.
    *
    *******************************************************************************/
    #define mSPI_SpiIsBusBusy() ((uint32) (0u != (mSPI_SPI_STATUS_REG & \
                                                              mSPI_SPI_STATUS_BUS_BUSY)))

    #if (mSPI_SPI_MASTER_CONST)
        void mSPI_SpiSetActiveSlaveSelect(uint32 slaveSelect);
    #endif /*(mSPI_SPI_MASTER_CONST) */

    #if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
        void mSPI_SpiSetSlaveSelectPolarity(uint32 slaveSelect, uint32 polarity);
    #endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
#endif /* (mSPI_SCB_MODE_SPI_INC) */
/** @} spi */

/**
* \addtogroup group_uart
* @{
*/
/* UART specific functions */
#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    void mSPI_UartInit(const mSPI_UART_INIT_STRUCT *config);
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

#if(mSPI_SCB_MODE_UART_INC)
    void mSPI_UartSetRxAddress(uint32 address);
    void mSPI_UartSetRxAddressMask(uint32 addressMask);


    /* UART RX direction APIs */
    #if(mSPI_UART_RX_DIRECTION)
        uint32 mSPI_UartGetChar(void);
        uint32 mSPI_UartGetByte(void);

        #if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void mSPI_UartSetRtsPolarity(uint32 polarity);
            void mSPI_UartSetRtsFifoLevel(uint32 level);
        #endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
    #endif /* (mSPI_UART_RX_DIRECTION) */

    /* UART TX direction APIs */
    #if(mSPI_UART_TX_DIRECTION)
        /*******************************************************************************
        * Function Name: mSPI_UartPutChar
        ****************************************************************************//**
        *
        *  Places a byte of data in the transmit buffer to be sent at the next available
        *  bus time. This function is blocking and waits until there is a space
        *  available to put requested data in the transmit buffer.
        *  For UART Multi Processor mode this function can send 9-bits data as well.
        *  Use mSPI_UART_MP_MARK to add a mark to create an address byte.
        *
        *  \param txDataByte: the data to be transmitted.
        *
        *******************************************************************************/
        #define mSPI_UartPutChar(ch)    mSPI_SpiUartWriteTxData((uint32)(ch))

        void mSPI_UartPutString(const char8 string[]);
        void mSPI_UartPutCRLF(uint32 txDataByte);
        void mSPI_UartSendBreakBlocking(uint32 breakWidth);

        #if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
            /* UART APIs for Flow Control */
            void mSPI_UartEnableCts(void);
            void mSPI_UartDisableCts(void);
            void mSPI_UartSetCtsPolarity(uint32 polarity);
        #endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
    #endif /* (mSPI_UART_TX_DIRECTION) */
#endif /* (mSPI_SCB_MODE_UART_INC) */
/** @} uart */

/**
* \addtogroup group_spi_uart
* @{
*/
#if(mSPI_RX_DIRECTION)
    uint32 mSPI_SpiUartReadRxData(void);
    uint32 mSPI_SpiUartGetRxBufferSize(void);
    void   mSPI_SpiUartClearRxBuffer(void);
#endif /* (mSPI_RX_DIRECTION) */

/* Common APIs TX direction */
#if(mSPI_TX_DIRECTION)
    void   mSPI_SpiUartWriteTxData(uint32 txData);
    void   mSPI_SpiUartPutArray(const uint8 wrBuf[], uint32 count);
    uint32 mSPI_SpiUartGetTxBufferSize(void);
    void   mSPI_SpiUartClearTxBuffer(void);
#endif /* (mSPI_TX_DIRECTION) */
/** @} spi_uart */

CY_ISR_PROTO(mSPI_SPI_UART_ISR);

#if(mSPI_UART_RX_WAKEUP_IRQ)
    CY_ISR_PROTO(mSPI_UART_WAKEUP_ISR);
#endif /* (mSPI_UART_RX_WAKEUP_IRQ) */


/***************************************
*     Buffer Access Macro Definitions
***************************************/

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    /* RX direction */
    void   mSPI_PutWordInRxBuffer  (uint32 idx, uint32 rxDataByte);
    uint32 mSPI_GetWordFromRxBuffer(uint32 idx);

    /* TX direction */
    void   mSPI_PutWordInTxBuffer  (uint32 idx, uint32 txDataByte);
    uint32 mSPI_GetWordFromTxBuffer(uint32 idx);

#else
    /* RX direction */
    #if(mSPI_INTERNAL_RX_SW_BUFFER_CONST)
        #define mSPI_PutWordInRxBuffer(idx, rxDataByte) \
                do{                                                 \
                    mSPI_rxBufferInternal[(idx)] = ((uint8) (rxDataByte)); \
                }while(0)

        #define mSPI_GetWordFromRxBuffer(idx) mSPI_rxBufferInternal[(idx)]

    #endif /* (mSPI_INTERNAL_RX_SW_BUFFER_CONST) */

    /* TX direction */
    #if(mSPI_INTERNAL_TX_SW_BUFFER_CONST)
        #define mSPI_PutWordInTxBuffer(idx, txDataByte) \
                    do{                                             \
                        mSPI_txBufferInternal[(idx)] = ((uint8) (txDataByte)); \
                    }while(0)

        #define mSPI_GetWordFromTxBuffer(idx) mSPI_txBufferInternal[(idx)]

    #endif /* (mSPI_INTERNAL_TX_SW_BUFFER_CONST) */

#endif /* (mSPI_TX_SW_BUFFER_ENABLE) */


/***************************************
*         SPI API Constants
***************************************/

/* SPI sub mode enum */
#define mSPI_SPI_MODE_MOTOROLA      (0x00u)
#define mSPI_SPI_MODE_TI_COINCIDES  (0x01u)
#define mSPI_SPI_MODE_TI_PRECEDES   (0x11u)
#define mSPI_SPI_MODE_NATIONAL      (0x02u)
#define mSPI_SPI_MODE_MASK          (0x03u)
#define mSPI_SPI_MODE_TI_PRECEDES_MASK  (0x10u)
#define mSPI_SPI_MODE_NS_MICROWIRE  (mSPI_SPI_MODE_NATIONAL)

/* SPI phase and polarity mode enum */
#define mSPI_SPI_SCLK_CPHA0_CPOL0   (0x00u)
#define mSPI_SPI_SCLK_CPHA0_CPOL1   (0x02u)
#define mSPI_SPI_SCLK_CPHA1_CPOL0   (0x01u)
#define mSPI_SPI_SCLK_CPHA1_CPOL1   (0x03u)

/* SPI bits order enum */
#define mSPI_BITS_ORDER_LSB_FIRST   (0u)
#define mSPI_BITS_ORDER_MSB_FIRST   (1u)

/* SPI transfer separation enum */
#define mSPI_SPI_TRANSFER_SEPARATED     (0u)
#define mSPI_SPI_TRANSFER_CONTINUOUS    (1u)

/* SPI slave select constants */
#define mSPI_SPI_SLAVE_SELECT0    (mSPI_SCB__SS0_POSISTION)
#define mSPI_SPI_SLAVE_SELECT1    (mSPI_SCB__SS1_POSISTION)
#define mSPI_SPI_SLAVE_SELECT2    (mSPI_SCB__SS2_POSISTION)
#define mSPI_SPI_SLAVE_SELECT3    (mSPI_SCB__SS3_POSISTION)

/* SPI slave select polarity settings */
#define mSPI_SPI_SS_ACTIVE_LOW  (0u)
#define mSPI_SPI_SS_ACTIVE_HIGH (1u)

#define mSPI_INTR_SPIM_TX_RESTORE   (mSPI_INTR_TX_OVERFLOW)

#define mSPI_INTR_SPIS_TX_RESTORE     (mSPI_INTR_TX_OVERFLOW | \
                                                 mSPI_INTR_TX_UNDERFLOW)

/***************************************
*         UART API Constants
***************************************/

/* UART sub-modes enum */
#define mSPI_UART_MODE_STD          (0u)
#define mSPI_UART_MODE_SMARTCARD    (1u)
#define mSPI_UART_MODE_IRDA         (2u)

/* UART direction enum */
#define mSPI_UART_RX    (1u)
#define mSPI_UART_TX    (2u)
#define mSPI_UART_TX_RX (3u)

/* UART parity enum */
#define mSPI_UART_PARITY_EVEN   (0u)
#define mSPI_UART_PARITY_ODD    (1u)
#define mSPI_UART_PARITY_NONE   (2u)

/* UART stop bits enum */
#define mSPI_UART_STOP_BITS_1   (2u)
#define mSPI_UART_STOP_BITS_1_5 (3u)
#define mSPI_UART_STOP_BITS_2   (4u)

/* UART IrDA low power OVS enum */
#define mSPI_UART_IRDA_LP_OVS16     (16u)
#define mSPI_UART_IRDA_LP_OVS32     (32u)
#define mSPI_UART_IRDA_LP_OVS48     (48u)
#define mSPI_UART_IRDA_LP_OVS96     (96u)
#define mSPI_UART_IRDA_LP_OVS192    (192u)
#define mSPI_UART_IRDA_LP_OVS768    (768u)
#define mSPI_UART_IRDA_LP_OVS1536   (1536u)

/* Uart MP: mark (address) and space (data) bit definitions */
#define mSPI_UART_MP_MARK       (0x100u)
#define mSPI_UART_MP_SPACE      (0x000u)

/* UART CTS/RTS polarity settings */
#define mSPI_UART_CTS_ACTIVE_LOW    (0u)
#define mSPI_UART_CTS_ACTIVE_HIGH   (1u)
#define mSPI_UART_RTS_ACTIVE_LOW    (0u)
#define mSPI_UART_RTS_ACTIVE_HIGH   (1u)

/* Sources of RX errors */
#define mSPI_INTR_RX_ERR        (mSPI_INTR_RX_OVERFLOW    | \
                                             mSPI_INTR_RX_UNDERFLOW   | \
                                             mSPI_INTR_RX_FRAME_ERROR | \
                                             mSPI_INTR_RX_PARITY_ERROR)

/* Shifted INTR_RX_ERR defines ONLY for mSPI_UartGetByte() */
#define mSPI_UART_RX_OVERFLOW       (mSPI_INTR_RX_OVERFLOW << 8u)
#define mSPI_UART_RX_UNDERFLOW      (mSPI_INTR_RX_UNDERFLOW << 8u)
#define mSPI_UART_RX_FRAME_ERROR    (mSPI_INTR_RX_FRAME_ERROR << 8u)
#define mSPI_UART_RX_PARITY_ERROR   (mSPI_INTR_RX_PARITY_ERROR << 8u)
#define mSPI_UART_RX_ERROR_MASK     (mSPI_UART_RX_OVERFLOW    | \
                                                 mSPI_UART_RX_UNDERFLOW   | \
                                                 mSPI_UART_RX_FRAME_ERROR | \
                                                 mSPI_UART_RX_PARITY_ERROR)

#define mSPI_INTR_UART_TX_RESTORE   (mSPI_INTR_TX_OVERFLOW  | \
                                                 mSPI_INTR_TX_UART_NACK | \
                                                 mSPI_INTR_TX_UART_DONE | \
                                                 mSPI_INTR_TX_UART_ARB_LOST)


/***************************************
*     Vars with External Linkage
***************************************/

#if(mSPI_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const mSPI_SPI_INIT_STRUCT  mSPI_configSpi;
    extern const mSPI_UART_INIT_STRUCT mSPI_configUart;
#endif /* (mSPI_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (mSPI_UART_WAKE_ENABLE_CONST && mSPI_UART_RX_WAKEUP_IRQ)
    extern uint8 mSPI_skipStart;
#endif /* (mSPI_UART_WAKE_ENABLE_CONST && mSPI_UART_RX_WAKEUP_IRQ) */


/***************************************
*    Specific SPI Macro Definitions
***************************************/

#define mSPI_GET_SPI_INTR_SLAVE_MASK(sourceMask)  ((sourceMask) & mSPI_INTR_SLAVE_SPI_BUS_ERROR)
#define mSPI_GET_SPI_INTR_MASTER_MASK(sourceMask) ((sourceMask) & mSPI_INTR_MASTER_SPI_DONE)
#define mSPI_GET_SPI_INTR_RX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~mSPI_INTR_SLAVE_SPI_BUS_ERROR)

#define mSPI_GET_SPI_INTR_TX_MASK(sourceMask) \
                                             ((sourceMask) & (uint32) ~mSPI_INTR_MASTER_SPI_DONE)


/***************************************
*    Specific UART Macro Definitions
***************************************/

#define mSPI_UART_GET_CTRL_OVS_IRDA_LP(oversample) \
        ((mSPI_UART_IRDA_LP_OVS16   == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS16 : \
         ((mSPI_UART_IRDA_LP_OVS32   == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS32 : \
          ((mSPI_UART_IRDA_LP_OVS48   == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS48 : \
           ((mSPI_UART_IRDA_LP_OVS96   == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS96 : \
            ((mSPI_UART_IRDA_LP_OVS192  == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS192 : \
             ((mSPI_UART_IRDA_LP_OVS768  == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS768 : \
              ((mSPI_UART_IRDA_LP_OVS1536 == (oversample)) ? mSPI_CTRL_OVS_IRDA_LP_OVS1536 : \
                                                                          mSPI_CTRL_OVS_IRDA_LP_OVS16)))))))

#define mSPI_GET_UART_RX_CTRL_ENABLED(direction) ((0u != (mSPI_UART_RX & (direction))) ? \
                                                                     (mSPI_RX_CTRL_ENABLED) : (0u))

#define mSPI_GET_UART_TX_CTRL_ENABLED(direction) ((0u != (mSPI_UART_TX & (direction))) ? \
                                                                     (mSPI_TX_CTRL_ENABLED) : (0u))


/***************************************
*        SPI Register Settings
***************************************/

#define mSPI_CTRL_SPI      (mSPI_CTRL_MODE_SPI)
#define mSPI_SPI_RX_CTRL   (mSPI_RX_CTRL_ENABLED)
#define mSPI_SPI_TX_CTRL   (mSPI_TX_CTRL_ENABLED)


/***************************************
*       SPI Init Register Settings
***************************************/

#define mSPI_SPI_SS_POLARITY \
             (((uint32) mSPI_SPI_SS0_POLARITY << mSPI_SPI_SLAVE_SELECT0) | \
              ((uint32) mSPI_SPI_SS1_POLARITY << mSPI_SPI_SLAVE_SELECT1) | \
              ((uint32) mSPI_SPI_SS2_POLARITY << mSPI_SPI_SLAVE_SELECT2) | \
              ((uint32) mSPI_SPI_SS3_POLARITY << mSPI_SPI_SLAVE_SELECT3))

#if(mSPI_SCB_MODE_SPI_CONST_CFG)

    /* SPI Configuration */
    #define mSPI_SPI_DEFAULT_CTRL \
                    (mSPI_GET_CTRL_OVS(mSPI_SPI_OVS_FACTOR) | \
                     mSPI_GET_CTRL_BYTE_MODE (mSPI_SPI_BYTE_MODE_ENABLE) | \
                     mSPI_GET_CTRL_EC_AM_MODE(mSPI_SPI_WAKE_ENABLE)      | \
                     mSPI_CTRL_SPI)

    #define mSPI_SPI_DEFAULT_SPI_CTRL \
                    (mSPI_GET_SPI_CTRL_CONTINUOUS    (mSPI_SPI_TRANSFER_SEPARATION)       | \
                     mSPI_GET_SPI_CTRL_SELECT_PRECEDE(mSPI_SPI_SUB_MODE &                   \
                                                                  mSPI_SPI_MODE_TI_PRECEDES_MASK)     | \
                     mSPI_GET_SPI_CTRL_SCLK_MODE     (mSPI_SPI_CLOCK_MODE)                | \
                     mSPI_GET_SPI_CTRL_LATE_MISO_SAMPLE(mSPI_SPI_LATE_MISO_SAMPLE_ENABLE) | \
                     mSPI_GET_SPI_CTRL_SCLK_CONTINUOUS(mSPI_SPI_FREE_RUN_SCLK_ENABLE)     | \
                     mSPI_GET_SPI_CTRL_SSEL_POLARITY (mSPI_SPI_SS_POLARITY)               | \
                     mSPI_GET_SPI_CTRL_SUB_MODE      (mSPI_SPI_SUB_MODE)                  | \
                     mSPI_GET_SPI_CTRL_MASTER_MODE   (mSPI_SPI_MODE))

    /* RX direction */
    #define mSPI_SPI_DEFAULT_RX_CTRL \
                    (mSPI_GET_RX_CTRL_DATA_WIDTH(mSPI_SPI_RX_DATA_BITS_NUM)     | \
                     mSPI_GET_RX_CTRL_BIT_ORDER (mSPI_SPI_BITS_ORDER)           | \
                     mSPI_GET_RX_CTRL_MEDIAN    (mSPI_SPI_MEDIAN_FILTER_ENABLE) | \
                     mSPI_SPI_RX_CTRL)

    #define mSPI_SPI_DEFAULT_RX_FIFO_CTRL \
                    mSPI_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(mSPI_SPI_RX_TRIGGER_LEVEL)

    /* TX direction */
    #define mSPI_SPI_DEFAULT_TX_CTRL \
                    (mSPI_GET_TX_CTRL_DATA_WIDTH(mSPI_SPI_TX_DATA_BITS_NUM) | \
                     mSPI_GET_TX_CTRL_BIT_ORDER (mSPI_SPI_BITS_ORDER)       | \
                     mSPI_SPI_TX_CTRL)

    #define mSPI_SPI_DEFAULT_TX_FIFO_CTRL \
                    mSPI_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(mSPI_SPI_TX_TRIGGER_LEVEL)

    /* Interrupt sources */
    #define mSPI_SPI_DEFAULT_INTR_SPI_EC_MASK   (mSPI_NO_INTR_SOURCES)

    #define mSPI_SPI_DEFAULT_INTR_I2C_EC_MASK   (mSPI_NO_INTR_SOURCES)
    #define mSPI_SPI_DEFAULT_INTR_SLAVE_MASK \
                    (mSPI_SPI_INTR_RX_MASK & mSPI_INTR_SLAVE_SPI_BUS_ERROR)

    #define mSPI_SPI_DEFAULT_INTR_MASTER_MASK \
                    (mSPI_SPI_INTR_TX_MASK & mSPI_INTR_MASTER_SPI_DONE)

    #define mSPI_SPI_DEFAULT_INTR_RX_MASK \
                    (mSPI_SPI_INTR_RX_MASK & (uint32) ~mSPI_INTR_SLAVE_SPI_BUS_ERROR)

    #define mSPI_SPI_DEFAULT_INTR_TX_MASK \
                    (mSPI_SPI_INTR_TX_MASK & (uint32) ~mSPI_INTR_MASTER_SPI_DONE)

#endif /* (mSPI_SCB_MODE_SPI_CONST_CFG) */


/***************************************
*        UART Register Settings
***************************************/

#define mSPI_CTRL_UART      (mSPI_CTRL_MODE_UART)
#define mSPI_UART_RX_CTRL   (mSPI_RX_CTRL_LSB_FIRST) /* LSB for UART goes first */
#define mSPI_UART_TX_CTRL   (mSPI_TX_CTRL_LSB_FIRST) /* LSB for UART goes first */


/***************************************
*      UART Init Register Settings
***************************************/

#if(mSPI_SCB_MODE_UART_CONST_CFG)

    /* UART configuration */
    #if(mSPI_UART_MODE_IRDA == mSPI_UART_SUB_MODE)

        #define mSPI_DEFAULT_CTRL_OVS   ((0u != mSPI_UART_IRDA_LOW_POWER) ?              \
                                (mSPI_UART_GET_CTRL_OVS_IRDA_LP(mSPI_UART_OVS_FACTOR)) : \
                                (mSPI_CTRL_OVS_IRDA_OVS16))

    #else

        #define mSPI_DEFAULT_CTRL_OVS   mSPI_GET_CTRL_OVS(mSPI_UART_OVS_FACTOR)

    #endif /* (mSPI_UART_MODE_IRDA == mSPI_UART_SUB_MODE) */

    #define mSPI_UART_DEFAULT_CTRL \
                                (mSPI_GET_CTRL_BYTE_MODE  (mSPI_UART_BYTE_MODE_ENABLE)  | \
                                 mSPI_GET_CTRL_ADDR_ACCEPT(mSPI_UART_MP_ACCEPT_ADDRESS) | \
                                 mSPI_DEFAULT_CTRL_OVS                                              | \
                                 mSPI_CTRL_UART)

    #define mSPI_UART_DEFAULT_UART_CTRL \
                                    (mSPI_GET_UART_CTRL_MODE(mSPI_UART_SUB_MODE))

    /* RX direction */
    #define mSPI_UART_DEFAULT_RX_CTRL_PARITY \
                                ((mSPI_UART_PARITY_NONE != mSPI_UART_PARITY_TYPE) ?      \
                                  (mSPI_GET_UART_RX_CTRL_PARITY(mSPI_UART_PARITY_TYPE) | \
                                   mSPI_UART_RX_CTRL_PARITY_ENABLED) : (0u))

    #define mSPI_UART_DEFAULT_UART_RX_CTRL \
                    (mSPI_GET_UART_RX_CTRL_MODE(mSPI_UART_STOP_BITS_NUM)                    | \
                     mSPI_GET_UART_RX_CTRL_POLARITY(mSPI_UART_IRDA_POLARITY)                | \
                     mSPI_GET_UART_RX_CTRL_MP_MODE(mSPI_UART_MP_MODE_ENABLE)                | \
                     mSPI_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(mSPI_UART_DROP_ON_PARITY_ERR) | \
                     mSPI_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(mSPI_UART_DROP_ON_FRAME_ERR)   | \
                     mSPI_GET_UART_RX_CTRL_BREAK_WIDTH(mSPI_UART_RX_BREAK_WIDTH)            | \
                     mSPI_UART_DEFAULT_RX_CTRL_PARITY)


    #define mSPI_UART_DEFAULT_RX_CTRL \
                                (mSPI_GET_RX_CTRL_DATA_WIDTH(mSPI_UART_DATA_BITS_NUM)        | \
                                 mSPI_GET_RX_CTRL_MEDIAN    (mSPI_UART_MEDIAN_FILTER_ENABLE) | \
                                 mSPI_GET_UART_RX_CTRL_ENABLED(mSPI_UART_DIRECTION))

    #define mSPI_UART_DEFAULT_RX_FIFO_CTRL \
                                mSPI_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(mSPI_UART_RX_TRIGGER_LEVEL)

    #define mSPI_UART_DEFAULT_RX_MATCH_REG  ((0u != mSPI_UART_MP_MODE_ENABLE) ?          \
                                (mSPI_GET_RX_MATCH_ADDR(mSPI_UART_MP_RX_ADDRESS) | \
                                 mSPI_GET_RX_MATCH_MASK(mSPI_UART_MP_RX_ADDRESS_MASK)) : (0u))

    /* TX direction */
    #define mSPI_UART_DEFAULT_TX_CTRL_PARITY (mSPI_UART_DEFAULT_RX_CTRL_PARITY)

    #define mSPI_UART_DEFAULT_UART_TX_CTRL \
                                (mSPI_GET_UART_TX_CTRL_MODE(mSPI_UART_STOP_BITS_NUM)       | \
                                 mSPI_GET_UART_TX_CTRL_RETRY_NACK(mSPI_UART_RETRY_ON_NACK) | \
                                 mSPI_UART_DEFAULT_TX_CTRL_PARITY)

    #define mSPI_UART_DEFAULT_TX_CTRL \
                                (mSPI_GET_TX_CTRL_DATA_WIDTH(mSPI_UART_DATA_BITS_NUM) | \
                                 mSPI_GET_UART_TX_CTRL_ENABLED(mSPI_UART_DIRECTION))

    #define mSPI_UART_DEFAULT_TX_FIFO_CTRL \
                                mSPI_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(mSPI_UART_TX_TRIGGER_LEVEL)

    #define mSPI_UART_DEFAULT_FLOW_CTRL \
                        (mSPI_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(mSPI_UART_RTS_FIFO_LEVEL) | \
                         mSPI_GET_UART_FLOW_CTRL_RTS_POLARITY (mSPI_UART_RTS_POLARITY)   | \
                         mSPI_GET_UART_FLOW_CTRL_CTS_POLARITY (mSPI_UART_CTS_POLARITY)   | \
                         mSPI_GET_UART_FLOW_CTRL_CTS_ENABLE   (mSPI_UART_CTS_ENABLE))

    /* Interrupt sources */
    #define mSPI_UART_DEFAULT_INTR_I2C_EC_MASK  (mSPI_NO_INTR_SOURCES)
    #define mSPI_UART_DEFAULT_INTR_SPI_EC_MASK  (mSPI_NO_INTR_SOURCES)
    #define mSPI_UART_DEFAULT_INTR_SLAVE_MASK   (mSPI_NO_INTR_SOURCES)
    #define mSPI_UART_DEFAULT_INTR_MASTER_MASK  (mSPI_NO_INTR_SOURCES)
    #define mSPI_UART_DEFAULT_INTR_RX_MASK      (mSPI_UART_INTR_RX_MASK)
    #define mSPI_UART_DEFAULT_INTR_TX_MASK      (mSPI_UART_INTR_TX_MASK)

#endif /* (mSPI_SCB_MODE_UART_CONST_CFG) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

#define mSPI_SPIM_ACTIVE_SS0    (mSPI_SPI_SLAVE_SELECT0)
#define mSPI_SPIM_ACTIVE_SS1    (mSPI_SPI_SLAVE_SELECT1)
#define mSPI_SPIM_ACTIVE_SS2    (mSPI_SPI_SLAVE_SELECT2)
#define mSPI_SPIM_ACTIVE_SS3    (mSPI_SPI_SLAVE_SELECT3)

#endif /* CY_SCB_SPI_UART_mSPI_H */


/* [] END OF FILE */
