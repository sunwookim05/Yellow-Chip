/***************************************************************************//**
* \file mUART_PINS.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
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

#if !defined(CY_SCB_PINS_mUART_H)
#define CY_SCB_PINS_mUART_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define mUART_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define mUART_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define mUART_REMOVE_TX_SDA_MISO_PIN      (1u)
#define mUART_REMOVE_CTS_SCLK_PIN      (1u)
#define mUART_REMOVE_RTS_SS0_PIN      (1u)
#define mUART_REMOVE_SS1_PIN                 (1u)
#define mUART_REMOVE_SS2_PIN                 (1u)
#define mUART_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define mUART_REMOVE_I2C_PINS                (1u)
#define mUART_REMOVE_SPI_MASTER_PINS         (1u)
#define mUART_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define mUART_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define mUART_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define mUART_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define mUART_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define mUART_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define mUART_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define mUART_REMOVE_SPI_SLAVE_PINS          (1u)
#define mUART_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define mUART_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define mUART_REMOVE_UART_TX_PIN             (0u)
#define mUART_REMOVE_UART_RX_TX_PIN          (1u)
#define mUART_REMOVE_UART_RX_PIN             (0u)
#define mUART_REMOVE_UART_RX_WAKE_PIN        (1u)
#define mUART_REMOVE_UART_RTS_PIN            (1u)
#define mUART_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define mUART_RX_WAKE_SCL_MOSI_PIN (0u == mUART_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define mUART_RX_SCL_MOSI_PIN     (0u == mUART_REMOVE_RX_SCL_MOSI_PIN)
#define mUART_TX_SDA_MISO_PIN     (0u == mUART_REMOVE_TX_SDA_MISO_PIN)
#define mUART_CTS_SCLK_PIN     (0u == mUART_REMOVE_CTS_SCLK_PIN)
#define mUART_RTS_SS0_PIN     (0u == mUART_REMOVE_RTS_SS0_PIN)
#define mUART_SS1_PIN                (0u == mUART_REMOVE_SS1_PIN)
#define mUART_SS2_PIN                (0u == mUART_REMOVE_SS2_PIN)
#define mUART_SS3_PIN                (0u == mUART_REMOVE_SS3_PIN)

/* Mode defined pins */
#define mUART_I2C_PINS               (0u == mUART_REMOVE_I2C_PINS)
#define mUART_SPI_MASTER_PINS        (0u == mUART_REMOVE_SPI_MASTER_PINS)
#define mUART_SPI_MASTER_SCLK_PIN    (0u == mUART_REMOVE_SPI_MASTER_SCLK_PIN)
#define mUART_SPI_MASTER_MOSI_PIN    (0u == mUART_REMOVE_SPI_MASTER_MOSI_PIN)
#define mUART_SPI_MASTER_MISO_PIN    (0u == mUART_REMOVE_SPI_MASTER_MISO_PIN)
#define mUART_SPI_MASTER_SS0_PIN     (0u == mUART_REMOVE_SPI_MASTER_SS0_PIN)
#define mUART_SPI_MASTER_SS1_PIN     (0u == mUART_REMOVE_SPI_MASTER_SS1_PIN)
#define mUART_SPI_MASTER_SS2_PIN     (0u == mUART_REMOVE_SPI_MASTER_SS2_PIN)
#define mUART_SPI_MASTER_SS3_PIN     (0u == mUART_REMOVE_SPI_MASTER_SS3_PIN)
#define mUART_SPI_SLAVE_PINS         (0u == mUART_REMOVE_SPI_SLAVE_PINS)
#define mUART_SPI_SLAVE_MOSI_PIN     (0u == mUART_REMOVE_SPI_SLAVE_MOSI_PIN)
#define mUART_SPI_SLAVE_MISO_PIN     (0u == mUART_REMOVE_SPI_SLAVE_MISO_PIN)
#define mUART_UART_TX_PIN            (0u == mUART_REMOVE_UART_TX_PIN)
#define mUART_UART_RX_TX_PIN         (0u == mUART_REMOVE_UART_RX_TX_PIN)
#define mUART_UART_RX_PIN            (0u == mUART_REMOVE_UART_RX_PIN)
#define mUART_UART_RX_WAKE_PIN       (0u == mUART_REMOVE_UART_RX_WAKE_PIN)
#define mUART_UART_RTS_PIN           (0u == mUART_REMOVE_UART_RTS_PIN)
#define mUART_UART_CTS_PIN           (0u == mUART_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (mUART_RX_WAKE_SCL_MOSI_PIN)
    #include "mUART_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (mUART_RX_SCL_MOSI) */

#if (mUART_RX_SCL_MOSI_PIN)
    #include "mUART_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (mUART_RX_SCL_MOSI) */

#if (mUART_TX_SDA_MISO_PIN)
    #include "mUART_uart_tx_i2c_sda_spi_miso.h"
#endif /* (mUART_TX_SDA_MISO) */

#if (mUART_CTS_SCLK_PIN)
    #include "mUART_uart_cts_spi_sclk.h"
#endif /* (mUART_CTS_SCLK) */

#if (mUART_RTS_SS0_PIN)
    #include "mUART_uart_rts_spi_ss0.h"
#endif /* (mUART_RTS_SS0_PIN) */

#if (mUART_SS1_PIN)
    #include "mUART_spi_ss1.h"
#endif /* (mUART_SS1_PIN) */

#if (mUART_SS2_PIN)
    #include "mUART_spi_ss2.h"
#endif /* (mUART_SS2_PIN) */

#if (mUART_SS3_PIN)
    #include "mUART_spi_ss3.h"
#endif /* (mUART_SS3_PIN) */

#if (mUART_I2C_PINS)
    #include "mUART_scl.h"
    #include "mUART_sda.h"
#endif /* (mUART_I2C_PINS) */

#if (mUART_SPI_MASTER_PINS)
#if (mUART_SPI_MASTER_SCLK_PIN)
    #include "mUART_sclk_m.h"
#endif /* (mUART_SPI_MASTER_SCLK_PIN) */

#if (mUART_SPI_MASTER_MOSI_PIN)
    #include "mUART_mosi_m.h"
#endif /* (mUART_SPI_MASTER_MOSI_PIN) */

#if (mUART_SPI_MASTER_MISO_PIN)
    #include "mUART_miso_m.h"
#endif /*(mUART_SPI_MASTER_MISO_PIN) */
#endif /* (mUART_SPI_MASTER_PINS) */

#if (mUART_SPI_SLAVE_PINS)
    #include "mUART_sclk_s.h"
    #include "mUART_ss_s.h"

#if (mUART_SPI_SLAVE_MOSI_PIN)
    #include "mUART_mosi_s.h"
#endif /* (mUART_SPI_SLAVE_MOSI_PIN) */

#if (mUART_SPI_SLAVE_MISO_PIN)
    #include "mUART_miso_s.h"
#endif /*(mUART_SPI_SLAVE_MISO_PIN) */
#endif /* (mUART_SPI_SLAVE_PINS) */

#if (mUART_SPI_MASTER_SS0_PIN)
    #include "mUART_ss0_m.h"
#endif /* (mUART_SPI_MASTER_SS0_PIN) */

#if (mUART_SPI_MASTER_SS1_PIN)
    #include "mUART_ss1_m.h"
#endif /* (mUART_SPI_MASTER_SS1_PIN) */

#if (mUART_SPI_MASTER_SS2_PIN)
    #include "mUART_ss2_m.h"
#endif /* (mUART_SPI_MASTER_SS2_PIN) */

#if (mUART_SPI_MASTER_SS3_PIN)
    #include "mUART_ss3_m.h"
#endif /* (mUART_SPI_MASTER_SS3_PIN) */

#if (mUART_UART_TX_PIN)
    #include "mUART_tx.h"
#endif /* (mUART_UART_TX_PIN) */

#if (mUART_UART_RX_TX_PIN)
    #include "mUART_rx_tx.h"
#endif /* (mUART_UART_RX_TX_PIN) */

#if (mUART_UART_RX_PIN)
    #include "mUART_rx.h"
#endif /* (mUART_UART_RX_PIN) */

#if (mUART_UART_RX_WAKE_PIN)
    #include "mUART_rx_wake.h"
#endif /* (mUART_UART_RX_WAKE_PIN) */

#if (mUART_UART_RTS_PIN)
    #include "mUART_rts.h"
#endif /* (mUART_UART_RTS_PIN) */

#if (mUART_UART_CTS_PIN)
    #include "mUART_cts.h"
#endif /* (mUART_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (mUART_RX_SCL_MOSI_PIN)
    #define mUART_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define mUART_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define mUART_RX_SCL_MOSI_HSIOM_MASK      (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define mUART_RX_SCL_MOSI_HSIOM_POS       (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define mUART_RX_SCL_MOSI_HSIOM_SEL_GPIO  (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define mUART_RX_SCL_MOSI_HSIOM_SEL_I2C   (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define mUART_RX_SCL_MOSI_HSIOM_SEL_SPI   (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define mUART_RX_SCL_MOSI_HSIOM_SEL_UART  (mUART_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (mUART_RX_WAKE_SCL_MOSI_PIN)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_MASK      (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_POS       (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define mUART_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (mUART_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define mUART_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) mUART_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define mUART_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) mUART_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define mUART_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (mUART_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define mUART_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) mUART_INTCFG_TYPE_MASK << \
                                                                           mUART_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins mUART_RX_SCL_MOSI_PIN or mUART_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (mUART_RX_SCL_MOSI_PIN) */

#if (mUART_TX_SDA_MISO_PIN)
    #define mUART_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define mUART_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define mUART_TX_SDA_MISO_HSIOM_MASK      (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define mUART_TX_SDA_MISO_HSIOM_POS       (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define mUART_TX_SDA_MISO_HSIOM_SEL_GPIO  (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define mUART_TX_SDA_MISO_HSIOM_SEL_I2C   (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define mUART_TX_SDA_MISO_HSIOM_SEL_SPI   (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define mUART_TX_SDA_MISO_HSIOM_SEL_UART  (mUART_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (mUART_TX_SDA_MISO_PIN) */

#if (mUART_CTS_SCLK_PIN)
    #define mUART_CTS_SCLK_HSIOM_REG   (*(reg32 *) mUART_uart_cts_spi_sclk__0__HSIOM)
    #define mUART_CTS_SCLK_HSIOM_PTR   ( (reg32 *) mUART_uart_cts_spi_sclk__0__HSIOM)
    
    #define mUART_CTS_SCLK_HSIOM_MASK      (mUART_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define mUART_CTS_SCLK_HSIOM_POS       (mUART_uart_cts_spi_sclk__0__HSIOM_SHIFT)
    #define mUART_CTS_SCLK_HSIOM_SEL_GPIO  (mUART_uart_cts_spi_sclk__0__HSIOM_GPIO)
    #define mUART_CTS_SCLK_HSIOM_SEL_I2C   (mUART_uart_cts_spi_sclk__0__HSIOM_I2C)
    #define mUART_CTS_SCLK_HSIOM_SEL_SPI   (mUART_uart_cts_spi_sclk__0__HSIOM_SPI)
    #define mUART_CTS_SCLK_HSIOM_SEL_UART  (mUART_uart_cts_spi_sclk__0__HSIOM_UART)
#endif /* (mUART_CTS_SCLK_PIN) */

#if (mUART_RTS_SS0_PIN)
    #define mUART_RTS_SS0_HSIOM_REG   (*(reg32 *) mUART_uart_rts_spi_ss0__0__HSIOM)
    #define mUART_RTS_SS0_HSIOM_PTR   ( (reg32 *) mUART_uart_rts_spi_ss0__0__HSIOM)
    
    #define mUART_RTS_SS0_HSIOM_MASK      (mUART_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define mUART_RTS_SS0_HSIOM_POS       (mUART_uart_rts_spi_ss0__0__HSIOM_SHIFT)
    #define mUART_RTS_SS0_HSIOM_SEL_GPIO  (mUART_uart_rts_spi_ss0__0__HSIOM_GPIO)
    #define mUART_RTS_SS0_HSIOM_SEL_I2C   (mUART_uart_rts_spi_ss0__0__HSIOM_I2C)
    #define mUART_RTS_SS0_HSIOM_SEL_SPI   (mUART_uart_rts_spi_ss0__0__HSIOM_SPI)
#if !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1)
    #define mUART_RTS_SS0_HSIOM_SEL_UART  (mUART_uart_rts_spi_ss0__0__HSIOM_UART)
#endif /* !(mUART_CY_SCBIP_V0 || mUART_CY_SCBIP_V1) */
#endif /* (mUART_RTS_SS0_PIN) */

#if (mUART_SS1_PIN)
    #define mUART_SS1_HSIOM_REG  (*(reg32 *) mUART_spi_ss1__0__HSIOM)
    #define mUART_SS1_HSIOM_PTR  ( (reg32 *) mUART_spi_ss1__0__HSIOM)
    
    #define mUART_SS1_HSIOM_MASK     (mUART_spi_ss1__0__HSIOM_MASK)
    #define mUART_SS1_HSIOM_POS      (mUART_spi_ss1__0__HSIOM_SHIFT)
    #define mUART_SS1_HSIOM_SEL_GPIO (mUART_spi_ss1__0__HSIOM_GPIO)
    #define mUART_SS1_HSIOM_SEL_I2C  (mUART_spi_ss1__0__HSIOM_I2C)
    #define mUART_SS1_HSIOM_SEL_SPI  (mUART_spi_ss1__0__HSIOM_SPI)
#endif /* (mUART_SS1_PIN) */

#if (mUART_SS2_PIN)
    #define mUART_SS2_HSIOM_REG     (*(reg32 *) mUART_spi_ss2__0__HSIOM)
    #define mUART_SS2_HSIOM_PTR     ( (reg32 *) mUART_spi_ss2__0__HSIOM)
    
    #define mUART_SS2_HSIOM_MASK     (mUART_spi_ss2__0__HSIOM_MASK)
    #define mUART_SS2_HSIOM_POS      (mUART_spi_ss2__0__HSIOM_SHIFT)
    #define mUART_SS2_HSIOM_SEL_GPIO (mUART_spi_ss2__0__HSIOM_GPIO)
    #define mUART_SS2_HSIOM_SEL_I2C  (mUART_spi_ss2__0__HSIOM_I2C)
    #define mUART_SS2_HSIOM_SEL_SPI  (mUART_spi_ss2__0__HSIOM_SPI)
#endif /* (mUART_SS2_PIN) */

#if (mUART_SS3_PIN)
    #define mUART_SS3_HSIOM_REG     (*(reg32 *) mUART_spi_ss3__0__HSIOM)
    #define mUART_SS3_HSIOM_PTR     ( (reg32 *) mUART_spi_ss3__0__HSIOM)
    
    #define mUART_SS3_HSIOM_MASK     (mUART_spi_ss3__0__HSIOM_MASK)
    #define mUART_SS3_HSIOM_POS      (mUART_spi_ss3__0__HSIOM_SHIFT)
    #define mUART_SS3_HSIOM_SEL_GPIO (mUART_spi_ss3__0__HSIOM_GPIO)
    #define mUART_SS3_HSIOM_SEL_I2C  (mUART_spi_ss3__0__HSIOM_I2C)
    #define mUART_SS3_HSIOM_SEL_SPI  (mUART_spi_ss3__0__HSIOM_SPI)
#endif /* (mUART_SS3_PIN) */

#if (mUART_I2C_PINS)
    #define mUART_SCL_HSIOM_REG  (*(reg32 *) mUART_scl__0__HSIOM)
    #define mUART_SCL_HSIOM_PTR  ( (reg32 *) mUART_scl__0__HSIOM)
    
    #define mUART_SCL_HSIOM_MASK     (mUART_scl__0__HSIOM_MASK)
    #define mUART_SCL_HSIOM_POS      (mUART_scl__0__HSIOM_SHIFT)
    #define mUART_SCL_HSIOM_SEL_GPIO (mUART_sda__0__HSIOM_GPIO)
    #define mUART_SCL_HSIOM_SEL_I2C  (mUART_sda__0__HSIOM_I2C)
    
    #define mUART_SDA_HSIOM_REG  (*(reg32 *) mUART_sda__0__HSIOM)
    #define mUART_SDA_HSIOM_PTR  ( (reg32 *) mUART_sda__0__HSIOM)
    
    #define mUART_SDA_HSIOM_MASK     (mUART_sda__0__HSIOM_MASK)
    #define mUART_SDA_HSIOM_POS      (mUART_sda__0__HSIOM_SHIFT)
    #define mUART_SDA_HSIOM_SEL_GPIO (mUART_sda__0__HSIOM_GPIO)
    #define mUART_SDA_HSIOM_SEL_I2C  (mUART_sda__0__HSIOM_I2C)
#endif /* (mUART_I2C_PINS) */

#if (mUART_SPI_SLAVE_PINS)
    #define mUART_SCLK_S_HSIOM_REG   (*(reg32 *) mUART_sclk_s__0__HSIOM)
    #define mUART_SCLK_S_HSIOM_PTR   ( (reg32 *) mUART_sclk_s__0__HSIOM)
    
    #define mUART_SCLK_S_HSIOM_MASK      (mUART_sclk_s__0__HSIOM_MASK)
    #define mUART_SCLK_S_HSIOM_POS       (mUART_sclk_s__0__HSIOM_SHIFT)
    #define mUART_SCLK_S_HSIOM_SEL_GPIO  (mUART_sclk_s__0__HSIOM_GPIO)
    #define mUART_SCLK_S_HSIOM_SEL_SPI   (mUART_sclk_s__0__HSIOM_SPI)
    
    #define mUART_SS0_S_HSIOM_REG    (*(reg32 *) mUART_ss0_s__0__HSIOM)
    #define mUART_SS0_S_HSIOM_PTR    ( (reg32 *) mUART_ss0_s__0__HSIOM)
    
    #define mUART_SS0_S_HSIOM_MASK       (mUART_ss0_s__0__HSIOM_MASK)
    #define mUART_SS0_S_HSIOM_POS        (mUART_ss0_s__0__HSIOM_SHIFT)
    #define mUART_SS0_S_HSIOM_SEL_GPIO   (mUART_ss0_s__0__HSIOM_GPIO)  
    #define mUART_SS0_S_HSIOM_SEL_SPI    (mUART_ss0_s__0__HSIOM_SPI)
#endif /* (mUART_SPI_SLAVE_PINS) */

#if (mUART_SPI_SLAVE_MOSI_PIN)
    #define mUART_MOSI_S_HSIOM_REG   (*(reg32 *) mUART_mosi_s__0__HSIOM)
    #define mUART_MOSI_S_HSIOM_PTR   ( (reg32 *) mUART_mosi_s__0__HSIOM)
    
    #define mUART_MOSI_S_HSIOM_MASK      (mUART_mosi_s__0__HSIOM_MASK)
    #define mUART_MOSI_S_HSIOM_POS       (mUART_mosi_s__0__HSIOM_SHIFT)
    #define mUART_MOSI_S_HSIOM_SEL_GPIO  (mUART_mosi_s__0__HSIOM_GPIO)
    #define mUART_MOSI_S_HSIOM_SEL_SPI   (mUART_mosi_s__0__HSIOM_SPI)
#endif /* (mUART_SPI_SLAVE_MOSI_PIN) */

#if (mUART_SPI_SLAVE_MISO_PIN)
    #define mUART_MISO_S_HSIOM_REG   (*(reg32 *) mUART_miso_s__0__HSIOM)
    #define mUART_MISO_S_HSIOM_PTR   ( (reg32 *) mUART_miso_s__0__HSIOM)
    
    #define mUART_MISO_S_HSIOM_MASK      (mUART_miso_s__0__HSIOM_MASK)
    #define mUART_MISO_S_HSIOM_POS       (mUART_miso_s__0__HSIOM_SHIFT)
    #define mUART_MISO_S_HSIOM_SEL_GPIO  (mUART_miso_s__0__HSIOM_GPIO)
    #define mUART_MISO_S_HSIOM_SEL_SPI   (mUART_miso_s__0__HSIOM_SPI)
#endif /* (mUART_SPI_SLAVE_MISO_PIN) */

#if (mUART_SPI_MASTER_MISO_PIN)
    #define mUART_MISO_M_HSIOM_REG   (*(reg32 *) mUART_miso_m__0__HSIOM)
    #define mUART_MISO_M_HSIOM_PTR   ( (reg32 *) mUART_miso_m__0__HSIOM)
    
    #define mUART_MISO_M_HSIOM_MASK      (mUART_miso_m__0__HSIOM_MASK)
    #define mUART_MISO_M_HSIOM_POS       (mUART_miso_m__0__HSIOM_SHIFT)
    #define mUART_MISO_M_HSIOM_SEL_GPIO  (mUART_miso_m__0__HSIOM_GPIO)
    #define mUART_MISO_M_HSIOM_SEL_SPI   (mUART_miso_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_MISO_PIN) */

#if (mUART_SPI_MASTER_MOSI_PIN)
    #define mUART_MOSI_M_HSIOM_REG   (*(reg32 *) mUART_mosi_m__0__HSIOM)
    #define mUART_MOSI_M_HSIOM_PTR   ( (reg32 *) mUART_mosi_m__0__HSIOM)
    
    #define mUART_MOSI_M_HSIOM_MASK      (mUART_mosi_m__0__HSIOM_MASK)
    #define mUART_MOSI_M_HSIOM_POS       (mUART_mosi_m__0__HSIOM_SHIFT)
    #define mUART_MOSI_M_HSIOM_SEL_GPIO  (mUART_mosi_m__0__HSIOM_GPIO)
    #define mUART_MOSI_M_HSIOM_SEL_SPI   (mUART_mosi_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_MOSI_PIN) */

#if (mUART_SPI_MASTER_SCLK_PIN)
    #define mUART_SCLK_M_HSIOM_REG   (*(reg32 *) mUART_sclk_m__0__HSIOM)
    #define mUART_SCLK_M_HSIOM_PTR   ( (reg32 *) mUART_sclk_m__0__HSIOM)
    
    #define mUART_SCLK_M_HSIOM_MASK      (mUART_sclk_m__0__HSIOM_MASK)
    #define mUART_SCLK_M_HSIOM_POS       (mUART_sclk_m__0__HSIOM_SHIFT)
    #define mUART_SCLK_M_HSIOM_SEL_GPIO  (mUART_sclk_m__0__HSIOM_GPIO)
    #define mUART_SCLK_M_HSIOM_SEL_SPI   (mUART_sclk_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_SCLK_PIN) */

#if (mUART_SPI_MASTER_SS0_PIN)
    #define mUART_SS0_M_HSIOM_REG    (*(reg32 *) mUART_ss0_m__0__HSIOM)
    #define mUART_SS0_M_HSIOM_PTR    ( (reg32 *) mUART_ss0_m__0__HSIOM)
    
    #define mUART_SS0_M_HSIOM_MASK       (mUART_ss0_m__0__HSIOM_MASK)
    #define mUART_SS0_M_HSIOM_POS        (mUART_ss0_m__0__HSIOM_SHIFT)
    #define mUART_SS0_M_HSIOM_SEL_GPIO   (mUART_ss0_m__0__HSIOM_GPIO)
    #define mUART_SS0_M_HSIOM_SEL_SPI    (mUART_ss0_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_SS0_PIN) */

#if (mUART_SPI_MASTER_SS1_PIN)
    #define mUART_SS1_M_HSIOM_REG    (*(reg32 *) mUART_ss1_m__0__HSIOM)
    #define mUART_SS1_M_HSIOM_PTR    ( (reg32 *) mUART_ss1_m__0__HSIOM)
    
    #define mUART_SS1_M_HSIOM_MASK       (mUART_ss1_m__0__HSIOM_MASK)
    #define mUART_SS1_M_HSIOM_POS        (mUART_ss1_m__0__HSIOM_SHIFT)
    #define mUART_SS1_M_HSIOM_SEL_GPIO   (mUART_ss1_m__0__HSIOM_GPIO)
    #define mUART_SS1_M_HSIOM_SEL_SPI    (mUART_ss1_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_SS1_PIN) */

#if (mUART_SPI_MASTER_SS2_PIN)
    #define mUART_SS2_M_HSIOM_REG    (*(reg32 *) mUART_ss2_m__0__HSIOM)
    #define mUART_SS2_M_HSIOM_PTR    ( (reg32 *) mUART_ss2_m__0__HSIOM)
    
    #define mUART_SS2_M_HSIOM_MASK       (mUART_ss2_m__0__HSIOM_MASK)
    #define mUART_SS2_M_HSIOM_POS        (mUART_ss2_m__0__HSIOM_SHIFT)
    #define mUART_SS2_M_HSIOM_SEL_GPIO   (mUART_ss2_m__0__HSIOM_GPIO)
    #define mUART_SS2_M_HSIOM_SEL_SPI    (mUART_ss2_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_SS2_PIN) */

#if (mUART_SPI_MASTER_SS3_PIN)
    #define mUART_SS3_M_HSIOM_REG    (*(reg32 *) mUART_ss3_m__0__HSIOM)
    #define mUART_SS3_M_HSIOM_PTR    ( (reg32 *) mUART_ss3_m__0__HSIOM)
    
    #define mUART_SS3_M_HSIOM_MASK      (mUART_ss3_m__0__HSIOM_MASK)
    #define mUART_SS3_M_HSIOM_POS       (mUART_ss3_m__0__HSIOM_SHIFT)
    #define mUART_SS3_M_HSIOM_SEL_GPIO  (mUART_ss3_m__0__HSIOM_GPIO)
    #define mUART_SS3_M_HSIOM_SEL_SPI   (mUART_ss3_m__0__HSIOM_SPI)
#endif /* (mUART_SPI_MASTER_SS3_PIN) */

#if (mUART_UART_RX_PIN)
    #define mUART_RX_HSIOM_REG   (*(reg32 *) mUART_rx__0__HSIOM)
    #define mUART_RX_HSIOM_PTR   ( (reg32 *) mUART_rx__0__HSIOM)
    
    #define mUART_RX_HSIOM_MASK      (mUART_rx__0__HSIOM_MASK)
    #define mUART_RX_HSIOM_POS       (mUART_rx__0__HSIOM_SHIFT)
    #define mUART_RX_HSIOM_SEL_GPIO  (mUART_rx__0__HSIOM_GPIO)
    #define mUART_RX_HSIOM_SEL_UART  (mUART_rx__0__HSIOM_UART)
#endif /* (mUART_UART_RX_PIN) */

#if (mUART_UART_RX_WAKE_PIN)
    #define mUART_RX_WAKE_HSIOM_REG   (*(reg32 *) mUART_rx_wake__0__HSIOM)
    #define mUART_RX_WAKE_HSIOM_PTR   ( (reg32 *) mUART_rx_wake__0__HSIOM)
    
    #define mUART_RX_WAKE_HSIOM_MASK      (mUART_rx_wake__0__HSIOM_MASK)
    #define mUART_RX_WAKE_HSIOM_POS       (mUART_rx_wake__0__HSIOM_SHIFT)
    #define mUART_RX_WAKE_HSIOM_SEL_GPIO  (mUART_rx_wake__0__HSIOM_GPIO)
    #define mUART_RX_WAKE_HSIOM_SEL_UART  (mUART_rx_wake__0__HSIOM_UART)
#endif /* (mUART_UART_WAKE_RX_PIN) */

#if (mUART_UART_CTS_PIN)
    #define mUART_CTS_HSIOM_REG   (*(reg32 *) mUART_cts__0__HSIOM)
    #define mUART_CTS_HSIOM_PTR   ( (reg32 *) mUART_cts__0__HSIOM)
    
    #define mUART_CTS_HSIOM_MASK      (mUART_cts__0__HSIOM_MASK)
    #define mUART_CTS_HSIOM_POS       (mUART_cts__0__HSIOM_SHIFT)
    #define mUART_CTS_HSIOM_SEL_GPIO  (mUART_cts__0__HSIOM_GPIO)
    #define mUART_CTS_HSIOM_SEL_UART  (mUART_cts__0__HSIOM_UART)
#endif /* (mUART_UART_CTS_PIN) */

#if (mUART_UART_TX_PIN)
    #define mUART_TX_HSIOM_REG   (*(reg32 *) mUART_tx__0__HSIOM)
    #define mUART_TX_HSIOM_PTR   ( (reg32 *) mUART_tx__0__HSIOM)
    
    #define mUART_TX_HSIOM_MASK      (mUART_tx__0__HSIOM_MASK)
    #define mUART_TX_HSIOM_POS       (mUART_tx__0__HSIOM_SHIFT)
    #define mUART_TX_HSIOM_SEL_GPIO  (mUART_tx__0__HSIOM_GPIO)
    #define mUART_TX_HSIOM_SEL_UART  (mUART_tx__0__HSIOM_UART)
#endif /* (mUART_UART_TX_PIN) */

#if (mUART_UART_RX_TX_PIN)
    #define mUART_RX_TX_HSIOM_REG   (*(reg32 *) mUART_rx_tx__0__HSIOM)
    #define mUART_RX_TX_HSIOM_PTR   ( (reg32 *) mUART_rx_tx__0__HSIOM)
    
    #define mUART_RX_TX_HSIOM_MASK      (mUART_rx_tx__0__HSIOM_MASK)
    #define mUART_RX_TX_HSIOM_POS       (mUART_rx_tx__0__HSIOM_SHIFT)
    #define mUART_RX_TX_HSIOM_SEL_GPIO  (mUART_rx_tx__0__HSIOM_GPIO)
    #define mUART_RX_TX_HSIOM_SEL_UART  (mUART_rx_tx__0__HSIOM_UART)
#endif /* (mUART_UART_RX_TX_PIN) */

#if (mUART_UART_RTS_PIN)
    #define mUART_RTS_HSIOM_REG      (*(reg32 *) mUART_rts__0__HSIOM)
    #define mUART_RTS_HSIOM_PTR      ( (reg32 *) mUART_rts__0__HSIOM)
    
    #define mUART_RTS_HSIOM_MASK     (mUART_rts__0__HSIOM_MASK)
    #define mUART_RTS_HSIOM_POS      (mUART_rts__0__HSIOM_SHIFT)    
    #define mUART_RTS_HSIOM_SEL_GPIO (mUART_rts__0__HSIOM_GPIO)
    #define mUART_RTS_HSIOM_SEL_UART (mUART_rts__0__HSIOM_UART)    
#endif /* (mUART_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define mUART_HSIOM_DEF_SEL      (0x00u)
#define mUART_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for mUART_CY_SCBIP_V0 
* and mUART_CY_SCBIP_V1. It is not recommended to use them for 
* mUART_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define mUART_HSIOM_UART_SEL     (0x09u)
#define mUART_HSIOM_I2C_SEL      (0x0Eu)
#define mUART_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define mUART_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define mUART_RX_SCL_MOSI_PIN_INDEX       (0u)
#define mUART_TX_SDA_MISO_PIN_INDEX       (1u)
#define mUART_CTS_SCLK_PIN_INDEX       (2u)
#define mUART_RTS_SS0_PIN_INDEX       (3u)
#define mUART_SS1_PIN_INDEX                  (4u)
#define mUART_SS2_PIN_INDEX                  (5u)
#define mUART_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define mUART_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << mUART_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define mUART_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << mUART_RX_SCL_MOSI_PIN_INDEX)
#define mUART_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << mUART_TX_SDA_MISO_PIN_INDEX)
#define mUART_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << mUART_CTS_SCLK_PIN_INDEX)
#define mUART_RTS_SS0_PIN_MASK     ((uint32) 0x01u << mUART_RTS_SS0_PIN_INDEX)
#define mUART_SS1_PIN_MASK                ((uint32) 0x01u << mUART_SS1_PIN_INDEX)
#define mUART_SS2_PIN_MASK                ((uint32) 0x01u << mUART_SS2_PIN_INDEX)
#define mUART_SS3_PIN_MASK                ((uint32) 0x01u << mUART_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define mUART_INTCFG_TYPE_MASK           (0x03u)
#define mUART_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define mUART_PIN_DM_ALG_HIZ  (0u)
#define mUART_PIN_DM_DIG_HIZ  (1u)
#define mUART_PIN_DM_OD_LO    (4u)
#define mUART_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define mUART_DM_MASK    (0x7u)
#define mUART_DM_SIZE    (3u)
#define mUART_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) mUART_DM_MASK << (mUART_DM_SIZE * (pos)))) >> \
                                                              (mUART_DM_SIZE * (pos)) )

#if (mUART_TX_SDA_MISO_PIN)
    #define mUART_CHECK_TX_SDA_MISO_PIN_USED \
                (mUART_PIN_DM_ALG_HIZ != \
                    mUART_GET_P4_PIN_DM(mUART_uart_tx_i2c_sda_spi_miso_PC, \
                                                   mUART_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (mUART_TX_SDA_MISO_PIN) */

#if (mUART_RTS_SS0_PIN)
    #define mUART_CHECK_RTS_SS0_PIN_USED \
                (mUART_PIN_DM_ALG_HIZ != \
                    mUART_GET_P4_PIN_DM(mUART_uart_rts_spi_ss0_PC, \
                                                   mUART_uart_rts_spi_ss0_SHIFT))
#endif /* (mUART_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define mUART_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define mUART_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define mUART_SET_HSIOM_SEL(reg, mask, pos, sel) mUART_SET_REGISTER_BITS(reg, mask, pos, sel)
#define mUART_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        mUART_SET_REGISTER_BITS(reg, mask, pos, intType)
#define mUART_SET_INP_DIS(reg, mask, val) mUART_SET_REGISTER_BIT(reg, mask, val)

/* mUART_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  mUART_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (mUART_CY_SCBIP_V0)
#if (mUART_I2C_PINS)
    #define mUART_SET_I2C_SCL_DR(val) mUART_scl_Write(val)

    #define mUART_SET_I2C_SCL_HSIOM_SEL(sel) \
                          mUART_SET_HSIOM_SEL(mUART_SCL_HSIOM_REG,  \
                                                         mUART_SCL_HSIOM_MASK, \
                                                         mUART_SCL_HSIOM_POS,  \
                                                         (sel))
    #define mUART_WAIT_SCL_SET_HIGH  (0u == mUART_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (mUART_RX_WAKE_SCL_MOSI_PIN)
    #define mUART_SET_I2C_SCL_DR(val) \
                            mUART_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define mUART_SET_I2C_SCL_HSIOM_SEL(sel) \
                    mUART_SET_HSIOM_SEL(mUART_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   mUART_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   mUART_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define mUART_WAIT_SCL_SET_HIGH  (0u == mUART_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (mUART_RX_SCL_MOSI_PIN)
    #define mUART_SET_I2C_SCL_DR(val) \
                            mUART_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define mUART_SET_I2C_SCL_HSIOM_SEL(sel) \
                            mUART_SET_HSIOM_SEL(mUART_RX_SCL_MOSI_HSIOM_REG,  \
                                                           mUART_RX_SCL_MOSI_HSIOM_MASK, \
                                                           mUART_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define mUART_WAIT_SCL_SET_HIGH  (0u == mUART_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define mUART_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define mUART_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define mUART_WAIT_SCL_SET_HIGH  (0u)
#endif /* (mUART_I2C_PINS) */

/* SCB I2C: sda signal */
#if (mUART_I2C_PINS)
    #define mUART_WAIT_SDA_SET_HIGH  (0u == mUART_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (mUART_TX_SDA_MISO_PIN)
    #define mUART_WAIT_SDA_SET_HIGH  (0u == mUART_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define mUART_WAIT_SDA_SET_HIGH  (0u)
#endif /* (mUART_MOSI_SCL_RX_PIN) */
#endif /* (mUART_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (mUART_RX_SCL_MOSI_PIN)
    #define mUART_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (mUART_RX_WAKE_SCL_MOSI_PIN)
    #define mUART_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) mUART_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(mUART_UART_RX_WAKE_PIN)
    #define mUART_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) mUART_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (mUART_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define mUART_REMOVE_MOSI_SCL_RX_WAKE_PIN    mUART_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define mUART_REMOVE_MOSI_SCL_RX_PIN         mUART_REMOVE_RX_SCL_MOSI_PIN
#define mUART_REMOVE_MISO_SDA_TX_PIN         mUART_REMOVE_TX_SDA_MISO_PIN
#ifndef mUART_REMOVE_SCLK_PIN
#define mUART_REMOVE_SCLK_PIN                mUART_REMOVE_CTS_SCLK_PIN
#endif /* mUART_REMOVE_SCLK_PIN */
#ifndef mUART_REMOVE_SS0_PIN
#define mUART_REMOVE_SS0_PIN                 mUART_REMOVE_RTS_SS0_PIN
#endif /* mUART_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define mUART_MOSI_SCL_RX_WAKE_PIN   mUART_RX_WAKE_SCL_MOSI_PIN
#define mUART_MOSI_SCL_RX_PIN        mUART_RX_SCL_MOSI_PIN
#define mUART_MISO_SDA_TX_PIN        mUART_TX_SDA_MISO_PIN
#ifndef mUART_SCLK_PIN
#define mUART_SCLK_PIN               mUART_CTS_SCLK_PIN
#endif /* mUART_SCLK_PIN */
#ifndef mUART_SS0_PIN
#define mUART_SS0_PIN                mUART_RTS_SS0_PIN
#endif /* mUART_SS0_PIN */

#if (mUART_MOSI_SCL_RX_WAKE_PIN)
    #define mUART_MOSI_SCL_RX_WAKE_HSIOM_REG     mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_WAKE_HSIOM_PTR     mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_WAKE_HSIOM_MASK    mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_WAKE_HSIOM_POS     mUART_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define mUART_MOSI_SCL_RX_WAKE_INTCFG_REG    mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_WAKE_INTCFG_PTR    mUART_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define mUART_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  mUART_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (mUART_RX_WAKE_SCL_MOSI_PIN) */

#if (mUART_MOSI_SCL_RX_PIN)
    #define mUART_MOSI_SCL_RX_HSIOM_REG      mUART_RX_SCL_MOSI_HSIOM_REG
    #define mUART_MOSI_SCL_RX_HSIOM_PTR      mUART_RX_SCL_MOSI_HSIOM_PTR
    #define mUART_MOSI_SCL_RX_HSIOM_MASK     mUART_RX_SCL_MOSI_HSIOM_MASK
    #define mUART_MOSI_SCL_RX_HSIOM_POS      mUART_RX_SCL_MOSI_HSIOM_POS
#endif /* (mUART_MOSI_SCL_RX_PIN) */

#if (mUART_MISO_SDA_TX_PIN)
    #define mUART_MISO_SDA_TX_HSIOM_REG      mUART_TX_SDA_MISO_HSIOM_REG
    #define mUART_MISO_SDA_TX_HSIOM_PTR      mUART_TX_SDA_MISO_HSIOM_REG
    #define mUART_MISO_SDA_TX_HSIOM_MASK     mUART_TX_SDA_MISO_HSIOM_REG
    #define mUART_MISO_SDA_TX_HSIOM_POS      mUART_TX_SDA_MISO_HSIOM_REG
#endif /* (mUART_MISO_SDA_TX_PIN_PIN) */

#if (mUART_SCLK_PIN)
    #ifndef mUART_SCLK_HSIOM_REG
    #define mUART_SCLK_HSIOM_REG     mUART_CTS_SCLK_HSIOM_REG
    #define mUART_SCLK_HSIOM_PTR     mUART_CTS_SCLK_HSIOM_PTR
    #define mUART_SCLK_HSIOM_MASK    mUART_CTS_SCLK_HSIOM_MASK
    #define mUART_SCLK_HSIOM_POS     mUART_CTS_SCLK_HSIOM_POS
    #endif /* mUART_SCLK_HSIOM_REG */
#endif /* (mUART_SCLK_PIN) */

#if (mUART_SS0_PIN)
    #ifndef mUART_SS0_HSIOM_REG
    #define mUART_SS0_HSIOM_REG      mUART_RTS_SS0_HSIOM_REG
    #define mUART_SS0_HSIOM_PTR      mUART_RTS_SS0_HSIOM_PTR
    #define mUART_SS0_HSIOM_MASK     mUART_RTS_SS0_HSIOM_MASK
    #define mUART_SS0_HSIOM_POS      mUART_RTS_SS0_HSIOM_POS
    #endif /* mUART_SS0_HSIOM_REG */
#endif /* (mUART_SS0_PIN) */

#define mUART_MOSI_SCL_RX_WAKE_PIN_INDEX mUART_RX_WAKE_SCL_MOSI_PIN_INDEX
#define mUART_MOSI_SCL_RX_PIN_INDEX      mUART_RX_SCL_MOSI_PIN_INDEX
#define mUART_MISO_SDA_TX_PIN_INDEX      mUART_TX_SDA_MISO_PIN_INDEX
#ifndef mUART_SCLK_PIN_INDEX
#define mUART_SCLK_PIN_INDEX             mUART_CTS_SCLK_PIN_INDEX
#endif /* mUART_SCLK_PIN_INDEX */
#ifndef mUART_SS0_PIN_INDEX
#define mUART_SS0_PIN_INDEX              mUART_RTS_SS0_PIN_INDEX
#endif /* mUART_SS0_PIN_INDEX */

#define mUART_MOSI_SCL_RX_WAKE_PIN_MASK mUART_RX_WAKE_SCL_MOSI_PIN_MASK
#define mUART_MOSI_SCL_RX_PIN_MASK      mUART_RX_SCL_MOSI_PIN_MASK
#define mUART_MISO_SDA_TX_PIN_MASK      mUART_TX_SDA_MISO_PIN_MASK
#ifndef mUART_SCLK_PIN_MASK
#define mUART_SCLK_PIN_MASK             mUART_CTS_SCLK_PIN_MASK
#endif /* mUART_SCLK_PIN_MASK */
#ifndef mUART_SS0_PIN_MASK
#define mUART_SS0_PIN_MASK              mUART_RTS_SS0_PIN_MASK
#endif /* mUART_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_mUART_H) */


/* [] END OF FILE */
