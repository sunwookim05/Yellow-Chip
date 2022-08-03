/***************************************************************************//**
* \file mSPI_PINS.h
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

#if !defined(CY_SCB_PINS_mSPI_H)
#define CY_SCB_PINS_mSPI_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define mSPI_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define mSPI_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define mSPI_REMOVE_TX_SDA_MISO_PIN      (1u)
#define mSPI_REMOVE_CTS_SCLK_PIN      (1u)
#define mSPI_REMOVE_RTS_SS0_PIN      (1u)
#define mSPI_REMOVE_SS1_PIN                 (1u)
#define mSPI_REMOVE_SS2_PIN                 (1u)
#define mSPI_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define mSPI_REMOVE_I2C_PINS                (1u)
#define mSPI_REMOVE_SPI_MASTER_PINS         (0u)
#define mSPI_REMOVE_SPI_MASTER_SCLK_PIN     (0u)
#define mSPI_REMOVE_SPI_MASTER_MOSI_PIN     (0u)
#define mSPI_REMOVE_SPI_MASTER_MISO_PIN     (0u)
#define mSPI_REMOVE_SPI_MASTER_SS0_PIN      (0u)
#define mSPI_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define mSPI_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define mSPI_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define mSPI_REMOVE_SPI_SLAVE_PINS          (1u)
#define mSPI_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define mSPI_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define mSPI_REMOVE_UART_TX_PIN             (1u)
#define mSPI_REMOVE_UART_RX_TX_PIN          (1u)
#define mSPI_REMOVE_UART_RX_PIN             (1u)
#define mSPI_REMOVE_UART_RX_WAKE_PIN        (1u)
#define mSPI_REMOVE_UART_RTS_PIN            (1u)
#define mSPI_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define mSPI_RX_WAKE_SCL_MOSI_PIN (0u == mSPI_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define mSPI_RX_SCL_MOSI_PIN     (0u == mSPI_REMOVE_RX_SCL_MOSI_PIN)
#define mSPI_TX_SDA_MISO_PIN     (0u == mSPI_REMOVE_TX_SDA_MISO_PIN)
#define mSPI_CTS_SCLK_PIN     (0u == mSPI_REMOVE_CTS_SCLK_PIN)
#define mSPI_RTS_SS0_PIN     (0u == mSPI_REMOVE_RTS_SS0_PIN)
#define mSPI_SS1_PIN                (0u == mSPI_REMOVE_SS1_PIN)
#define mSPI_SS2_PIN                (0u == mSPI_REMOVE_SS2_PIN)
#define mSPI_SS3_PIN                (0u == mSPI_REMOVE_SS3_PIN)

/* Mode defined pins */
#define mSPI_I2C_PINS               (0u == mSPI_REMOVE_I2C_PINS)
#define mSPI_SPI_MASTER_PINS        (0u == mSPI_REMOVE_SPI_MASTER_PINS)
#define mSPI_SPI_MASTER_SCLK_PIN    (0u == mSPI_REMOVE_SPI_MASTER_SCLK_PIN)
#define mSPI_SPI_MASTER_MOSI_PIN    (0u == mSPI_REMOVE_SPI_MASTER_MOSI_PIN)
#define mSPI_SPI_MASTER_MISO_PIN    (0u == mSPI_REMOVE_SPI_MASTER_MISO_PIN)
#define mSPI_SPI_MASTER_SS0_PIN     (0u == mSPI_REMOVE_SPI_MASTER_SS0_PIN)
#define mSPI_SPI_MASTER_SS1_PIN     (0u == mSPI_REMOVE_SPI_MASTER_SS1_PIN)
#define mSPI_SPI_MASTER_SS2_PIN     (0u == mSPI_REMOVE_SPI_MASTER_SS2_PIN)
#define mSPI_SPI_MASTER_SS3_PIN     (0u == mSPI_REMOVE_SPI_MASTER_SS3_PIN)
#define mSPI_SPI_SLAVE_PINS         (0u == mSPI_REMOVE_SPI_SLAVE_PINS)
#define mSPI_SPI_SLAVE_MOSI_PIN     (0u == mSPI_REMOVE_SPI_SLAVE_MOSI_PIN)
#define mSPI_SPI_SLAVE_MISO_PIN     (0u == mSPI_REMOVE_SPI_SLAVE_MISO_PIN)
#define mSPI_UART_TX_PIN            (0u == mSPI_REMOVE_UART_TX_PIN)
#define mSPI_UART_RX_TX_PIN         (0u == mSPI_REMOVE_UART_RX_TX_PIN)
#define mSPI_UART_RX_PIN            (0u == mSPI_REMOVE_UART_RX_PIN)
#define mSPI_UART_RX_WAKE_PIN       (0u == mSPI_REMOVE_UART_RX_WAKE_PIN)
#define mSPI_UART_RTS_PIN           (0u == mSPI_REMOVE_UART_RTS_PIN)
#define mSPI_UART_CTS_PIN           (0u == mSPI_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (mSPI_RX_WAKE_SCL_MOSI_PIN)
    #include "mSPI_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (mSPI_RX_SCL_MOSI) */

#if (mSPI_RX_SCL_MOSI_PIN)
    #include "mSPI_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (mSPI_RX_SCL_MOSI) */

#if (mSPI_TX_SDA_MISO_PIN)
    #include "mSPI_uart_tx_i2c_sda_spi_miso.h"
#endif /* (mSPI_TX_SDA_MISO) */

#if (mSPI_CTS_SCLK_PIN)
    #include "mSPI_uart_cts_spi_sclk.h"
#endif /* (mSPI_CTS_SCLK) */

#if (mSPI_RTS_SS0_PIN)
    #include "mSPI_uart_rts_spi_ss0.h"
#endif /* (mSPI_RTS_SS0_PIN) */

#if (mSPI_SS1_PIN)
    #include "mSPI_spi_ss1.h"
#endif /* (mSPI_SS1_PIN) */

#if (mSPI_SS2_PIN)
    #include "mSPI_spi_ss2.h"
#endif /* (mSPI_SS2_PIN) */

#if (mSPI_SS3_PIN)
    #include "mSPI_spi_ss3.h"
#endif /* (mSPI_SS3_PIN) */

#if (mSPI_I2C_PINS)
    #include "mSPI_scl.h"
    #include "mSPI_sda.h"
#endif /* (mSPI_I2C_PINS) */

#if (mSPI_SPI_MASTER_PINS)
#if (mSPI_SPI_MASTER_SCLK_PIN)
    #include "mSPI_sclk_m.h"
#endif /* (mSPI_SPI_MASTER_SCLK_PIN) */

#if (mSPI_SPI_MASTER_MOSI_PIN)
    #include "mSPI_mosi_m.h"
#endif /* (mSPI_SPI_MASTER_MOSI_PIN) */

#if (mSPI_SPI_MASTER_MISO_PIN)
    #include "mSPI_miso_m.h"
#endif /*(mSPI_SPI_MASTER_MISO_PIN) */
#endif /* (mSPI_SPI_MASTER_PINS) */

#if (mSPI_SPI_SLAVE_PINS)
    #include "mSPI_sclk_s.h"
    #include "mSPI_ss_s.h"

#if (mSPI_SPI_SLAVE_MOSI_PIN)
    #include "mSPI_mosi_s.h"
#endif /* (mSPI_SPI_SLAVE_MOSI_PIN) */

#if (mSPI_SPI_SLAVE_MISO_PIN)
    #include "mSPI_miso_s.h"
#endif /*(mSPI_SPI_SLAVE_MISO_PIN) */
#endif /* (mSPI_SPI_SLAVE_PINS) */

#if (mSPI_SPI_MASTER_SS0_PIN)
    #include "mSPI_ss0_m.h"
#endif /* (mSPI_SPI_MASTER_SS0_PIN) */

#if (mSPI_SPI_MASTER_SS1_PIN)
    #include "mSPI_ss1_m.h"
#endif /* (mSPI_SPI_MASTER_SS1_PIN) */

#if (mSPI_SPI_MASTER_SS2_PIN)
    #include "mSPI_ss2_m.h"
#endif /* (mSPI_SPI_MASTER_SS2_PIN) */

#if (mSPI_SPI_MASTER_SS3_PIN)
    #include "mSPI_ss3_m.h"
#endif /* (mSPI_SPI_MASTER_SS3_PIN) */

#if (mSPI_UART_TX_PIN)
    #include "mSPI_tx.h"
#endif /* (mSPI_UART_TX_PIN) */

#if (mSPI_UART_RX_TX_PIN)
    #include "mSPI_rx_tx.h"
#endif /* (mSPI_UART_RX_TX_PIN) */

#if (mSPI_UART_RX_PIN)
    #include "mSPI_rx.h"
#endif /* (mSPI_UART_RX_PIN) */

#if (mSPI_UART_RX_WAKE_PIN)
    #include "mSPI_rx_wake.h"
#endif /* (mSPI_UART_RX_WAKE_PIN) */

#if (mSPI_UART_RTS_PIN)
    #include "mSPI_rts.h"
#endif /* (mSPI_UART_RTS_PIN) */

#if (mSPI_UART_CTS_PIN)
    #include "mSPI_cts.h"
#endif /* (mSPI_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (mSPI_RX_SCL_MOSI_PIN)
    #define mSPI_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define mSPI_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define mSPI_RX_SCL_MOSI_HSIOM_MASK      (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define mSPI_RX_SCL_MOSI_HSIOM_POS       (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define mSPI_RX_SCL_MOSI_HSIOM_SEL_GPIO  (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define mSPI_RX_SCL_MOSI_HSIOM_SEL_I2C   (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define mSPI_RX_SCL_MOSI_HSIOM_SEL_SPI   (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define mSPI_RX_SCL_MOSI_HSIOM_SEL_UART  (mSPI_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_MASK      (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_POS       (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define mSPI_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define mSPI_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define mSPI_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) mSPI_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define mSPI_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (mSPI_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define mSPI_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) mSPI_INTCFG_TYPE_MASK << \
                                                                           mSPI_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins mSPI_RX_SCL_MOSI_PIN or mSPI_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (mSPI_RX_SCL_MOSI_PIN) */

#if (mSPI_TX_SDA_MISO_PIN)
    #define mSPI_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define mSPI_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define mSPI_TX_SDA_MISO_HSIOM_MASK      (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define mSPI_TX_SDA_MISO_HSIOM_POS       (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define mSPI_TX_SDA_MISO_HSIOM_SEL_GPIO  (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define mSPI_TX_SDA_MISO_HSIOM_SEL_I2C   (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define mSPI_TX_SDA_MISO_HSIOM_SEL_SPI   (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define mSPI_TX_SDA_MISO_HSIOM_SEL_UART  (mSPI_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (mSPI_TX_SDA_MISO_PIN) */

#if (mSPI_CTS_SCLK_PIN)
    #define mSPI_CTS_SCLK_HSIOM_REG   (*(reg32 *) mSPI_uart_cts_spi_sclk__0__HSIOM)
    #define mSPI_CTS_SCLK_HSIOM_PTR   ( (reg32 *) mSPI_uart_cts_spi_sclk__0__HSIOM)
    
    #define mSPI_CTS_SCLK_HSIOM_MASK      (mSPI_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define mSPI_CTS_SCLK_HSIOM_POS       (mSPI_uart_cts_spi_sclk__0__HSIOM_SHIFT)
    #define mSPI_CTS_SCLK_HSIOM_SEL_GPIO  (mSPI_uart_cts_spi_sclk__0__HSIOM_GPIO)
    #define mSPI_CTS_SCLK_HSIOM_SEL_I2C   (mSPI_uart_cts_spi_sclk__0__HSIOM_I2C)
    #define mSPI_CTS_SCLK_HSIOM_SEL_SPI   (mSPI_uart_cts_spi_sclk__0__HSIOM_SPI)
    #define mSPI_CTS_SCLK_HSIOM_SEL_UART  (mSPI_uart_cts_spi_sclk__0__HSIOM_UART)
#endif /* (mSPI_CTS_SCLK_PIN) */

#if (mSPI_RTS_SS0_PIN)
    #define mSPI_RTS_SS0_HSIOM_REG   (*(reg32 *) mSPI_uart_rts_spi_ss0__0__HSIOM)
    #define mSPI_RTS_SS0_HSIOM_PTR   ( (reg32 *) mSPI_uart_rts_spi_ss0__0__HSIOM)
    
    #define mSPI_RTS_SS0_HSIOM_MASK      (mSPI_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define mSPI_RTS_SS0_HSIOM_POS       (mSPI_uart_rts_spi_ss0__0__HSIOM_SHIFT)
    #define mSPI_RTS_SS0_HSIOM_SEL_GPIO  (mSPI_uart_rts_spi_ss0__0__HSIOM_GPIO)
    #define mSPI_RTS_SS0_HSIOM_SEL_I2C   (mSPI_uart_rts_spi_ss0__0__HSIOM_I2C)
    #define mSPI_RTS_SS0_HSIOM_SEL_SPI   (mSPI_uart_rts_spi_ss0__0__HSIOM_SPI)
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_RTS_SS0_HSIOM_SEL_UART  (mSPI_uart_rts_spi_ss0__0__HSIOM_UART)
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
#endif /* (mSPI_RTS_SS0_PIN) */

#if (mSPI_SS1_PIN)
    #define mSPI_SS1_HSIOM_REG  (*(reg32 *) mSPI_spi_ss1__0__HSIOM)
    #define mSPI_SS1_HSIOM_PTR  ( (reg32 *) mSPI_spi_ss1__0__HSIOM)
    
    #define mSPI_SS1_HSIOM_MASK     (mSPI_spi_ss1__0__HSIOM_MASK)
    #define mSPI_SS1_HSIOM_POS      (mSPI_spi_ss1__0__HSIOM_SHIFT)
    #define mSPI_SS1_HSIOM_SEL_GPIO (mSPI_spi_ss1__0__HSIOM_GPIO)
    #define mSPI_SS1_HSIOM_SEL_I2C  (mSPI_spi_ss1__0__HSIOM_I2C)
    #define mSPI_SS1_HSIOM_SEL_SPI  (mSPI_spi_ss1__0__HSIOM_SPI)
#endif /* (mSPI_SS1_PIN) */

#if (mSPI_SS2_PIN)
    #define mSPI_SS2_HSIOM_REG     (*(reg32 *) mSPI_spi_ss2__0__HSIOM)
    #define mSPI_SS2_HSIOM_PTR     ( (reg32 *) mSPI_spi_ss2__0__HSIOM)
    
    #define mSPI_SS2_HSIOM_MASK     (mSPI_spi_ss2__0__HSIOM_MASK)
    #define mSPI_SS2_HSIOM_POS      (mSPI_spi_ss2__0__HSIOM_SHIFT)
    #define mSPI_SS2_HSIOM_SEL_GPIO (mSPI_spi_ss2__0__HSIOM_GPIO)
    #define mSPI_SS2_HSIOM_SEL_I2C  (mSPI_spi_ss2__0__HSIOM_I2C)
    #define mSPI_SS2_HSIOM_SEL_SPI  (mSPI_spi_ss2__0__HSIOM_SPI)
#endif /* (mSPI_SS2_PIN) */

#if (mSPI_SS3_PIN)
    #define mSPI_SS3_HSIOM_REG     (*(reg32 *) mSPI_spi_ss3__0__HSIOM)
    #define mSPI_SS3_HSIOM_PTR     ( (reg32 *) mSPI_spi_ss3__0__HSIOM)
    
    #define mSPI_SS3_HSIOM_MASK     (mSPI_spi_ss3__0__HSIOM_MASK)
    #define mSPI_SS3_HSIOM_POS      (mSPI_spi_ss3__0__HSIOM_SHIFT)
    #define mSPI_SS3_HSIOM_SEL_GPIO (mSPI_spi_ss3__0__HSIOM_GPIO)
    #define mSPI_SS3_HSIOM_SEL_I2C  (mSPI_spi_ss3__0__HSIOM_I2C)
    #define mSPI_SS3_HSIOM_SEL_SPI  (mSPI_spi_ss3__0__HSIOM_SPI)
#endif /* (mSPI_SS3_PIN) */

#if (mSPI_I2C_PINS)
    #define mSPI_SCL_HSIOM_REG  (*(reg32 *) mSPI_scl__0__HSIOM)
    #define mSPI_SCL_HSIOM_PTR  ( (reg32 *) mSPI_scl__0__HSIOM)
    
    #define mSPI_SCL_HSIOM_MASK     (mSPI_scl__0__HSIOM_MASK)
    #define mSPI_SCL_HSIOM_POS      (mSPI_scl__0__HSIOM_SHIFT)
    #define mSPI_SCL_HSIOM_SEL_GPIO (mSPI_sda__0__HSIOM_GPIO)
    #define mSPI_SCL_HSIOM_SEL_I2C  (mSPI_sda__0__HSIOM_I2C)
    
    #define mSPI_SDA_HSIOM_REG  (*(reg32 *) mSPI_sda__0__HSIOM)
    #define mSPI_SDA_HSIOM_PTR  ( (reg32 *) mSPI_sda__0__HSIOM)
    
    #define mSPI_SDA_HSIOM_MASK     (mSPI_sda__0__HSIOM_MASK)
    #define mSPI_SDA_HSIOM_POS      (mSPI_sda__0__HSIOM_SHIFT)
    #define mSPI_SDA_HSIOM_SEL_GPIO (mSPI_sda__0__HSIOM_GPIO)
    #define mSPI_SDA_HSIOM_SEL_I2C  (mSPI_sda__0__HSIOM_I2C)
#endif /* (mSPI_I2C_PINS) */

#if (mSPI_SPI_SLAVE_PINS)
    #define mSPI_SCLK_S_HSIOM_REG   (*(reg32 *) mSPI_sclk_s__0__HSIOM)
    #define mSPI_SCLK_S_HSIOM_PTR   ( (reg32 *) mSPI_sclk_s__0__HSIOM)
    
    #define mSPI_SCLK_S_HSIOM_MASK      (mSPI_sclk_s__0__HSIOM_MASK)
    #define mSPI_SCLK_S_HSIOM_POS       (mSPI_sclk_s__0__HSIOM_SHIFT)
    #define mSPI_SCLK_S_HSIOM_SEL_GPIO  (mSPI_sclk_s__0__HSIOM_GPIO)
    #define mSPI_SCLK_S_HSIOM_SEL_SPI   (mSPI_sclk_s__0__HSIOM_SPI)
    
    #define mSPI_SS0_S_HSIOM_REG    (*(reg32 *) mSPI_ss0_s__0__HSIOM)
    #define mSPI_SS0_S_HSIOM_PTR    ( (reg32 *) mSPI_ss0_s__0__HSIOM)
    
    #define mSPI_SS0_S_HSIOM_MASK       (mSPI_ss0_s__0__HSIOM_MASK)
    #define mSPI_SS0_S_HSIOM_POS        (mSPI_ss0_s__0__HSIOM_SHIFT)
    #define mSPI_SS0_S_HSIOM_SEL_GPIO   (mSPI_ss0_s__0__HSIOM_GPIO)  
    #define mSPI_SS0_S_HSIOM_SEL_SPI    (mSPI_ss0_s__0__HSIOM_SPI)
#endif /* (mSPI_SPI_SLAVE_PINS) */

#if (mSPI_SPI_SLAVE_MOSI_PIN)
    #define mSPI_MOSI_S_HSIOM_REG   (*(reg32 *) mSPI_mosi_s__0__HSIOM)
    #define mSPI_MOSI_S_HSIOM_PTR   ( (reg32 *) mSPI_mosi_s__0__HSIOM)
    
    #define mSPI_MOSI_S_HSIOM_MASK      (mSPI_mosi_s__0__HSIOM_MASK)
    #define mSPI_MOSI_S_HSIOM_POS       (mSPI_mosi_s__0__HSIOM_SHIFT)
    #define mSPI_MOSI_S_HSIOM_SEL_GPIO  (mSPI_mosi_s__0__HSIOM_GPIO)
    #define mSPI_MOSI_S_HSIOM_SEL_SPI   (mSPI_mosi_s__0__HSIOM_SPI)
#endif /* (mSPI_SPI_SLAVE_MOSI_PIN) */

#if (mSPI_SPI_SLAVE_MISO_PIN)
    #define mSPI_MISO_S_HSIOM_REG   (*(reg32 *) mSPI_miso_s__0__HSIOM)
    #define mSPI_MISO_S_HSIOM_PTR   ( (reg32 *) mSPI_miso_s__0__HSIOM)
    
    #define mSPI_MISO_S_HSIOM_MASK      (mSPI_miso_s__0__HSIOM_MASK)
    #define mSPI_MISO_S_HSIOM_POS       (mSPI_miso_s__0__HSIOM_SHIFT)
    #define mSPI_MISO_S_HSIOM_SEL_GPIO  (mSPI_miso_s__0__HSIOM_GPIO)
    #define mSPI_MISO_S_HSIOM_SEL_SPI   (mSPI_miso_s__0__HSIOM_SPI)
#endif /* (mSPI_SPI_SLAVE_MISO_PIN) */

#if (mSPI_SPI_MASTER_MISO_PIN)
    #define mSPI_MISO_M_HSIOM_REG   (*(reg32 *) mSPI_miso_m__0__HSIOM)
    #define mSPI_MISO_M_HSIOM_PTR   ( (reg32 *) mSPI_miso_m__0__HSIOM)
    
    #define mSPI_MISO_M_HSIOM_MASK      (mSPI_miso_m__0__HSIOM_MASK)
    #define mSPI_MISO_M_HSIOM_POS       (mSPI_miso_m__0__HSIOM_SHIFT)
    #define mSPI_MISO_M_HSIOM_SEL_GPIO  (mSPI_miso_m__0__HSIOM_GPIO)
    #define mSPI_MISO_M_HSIOM_SEL_SPI   (mSPI_miso_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_MISO_PIN) */

#if (mSPI_SPI_MASTER_MOSI_PIN)
    #define mSPI_MOSI_M_HSIOM_REG   (*(reg32 *) mSPI_mosi_m__0__HSIOM)
    #define mSPI_MOSI_M_HSIOM_PTR   ( (reg32 *) mSPI_mosi_m__0__HSIOM)
    
    #define mSPI_MOSI_M_HSIOM_MASK      (mSPI_mosi_m__0__HSIOM_MASK)
    #define mSPI_MOSI_M_HSIOM_POS       (mSPI_mosi_m__0__HSIOM_SHIFT)
    #define mSPI_MOSI_M_HSIOM_SEL_GPIO  (mSPI_mosi_m__0__HSIOM_GPIO)
    #define mSPI_MOSI_M_HSIOM_SEL_SPI   (mSPI_mosi_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_MOSI_PIN) */

#if (mSPI_SPI_MASTER_SCLK_PIN)
    #define mSPI_SCLK_M_HSIOM_REG   (*(reg32 *) mSPI_sclk_m__0__HSIOM)
    #define mSPI_SCLK_M_HSIOM_PTR   ( (reg32 *) mSPI_sclk_m__0__HSIOM)
    
    #define mSPI_SCLK_M_HSIOM_MASK      (mSPI_sclk_m__0__HSIOM_MASK)
    #define mSPI_SCLK_M_HSIOM_POS       (mSPI_sclk_m__0__HSIOM_SHIFT)
    #define mSPI_SCLK_M_HSIOM_SEL_GPIO  (mSPI_sclk_m__0__HSIOM_GPIO)
    #define mSPI_SCLK_M_HSIOM_SEL_SPI   (mSPI_sclk_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_SCLK_PIN) */

#if (mSPI_SPI_MASTER_SS0_PIN)
    #define mSPI_SS0_M_HSIOM_REG    (*(reg32 *) mSPI_ss0_m__0__HSIOM)
    #define mSPI_SS0_M_HSIOM_PTR    ( (reg32 *) mSPI_ss0_m__0__HSIOM)
    
    #define mSPI_SS0_M_HSIOM_MASK       (mSPI_ss0_m__0__HSIOM_MASK)
    #define mSPI_SS0_M_HSIOM_POS        (mSPI_ss0_m__0__HSIOM_SHIFT)
    #define mSPI_SS0_M_HSIOM_SEL_GPIO   (mSPI_ss0_m__0__HSIOM_GPIO)
    #define mSPI_SS0_M_HSIOM_SEL_SPI    (mSPI_ss0_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_SS0_PIN) */

#if (mSPI_SPI_MASTER_SS1_PIN)
    #define mSPI_SS1_M_HSIOM_REG    (*(reg32 *) mSPI_ss1_m__0__HSIOM)
    #define mSPI_SS1_M_HSIOM_PTR    ( (reg32 *) mSPI_ss1_m__0__HSIOM)
    
    #define mSPI_SS1_M_HSIOM_MASK       (mSPI_ss1_m__0__HSIOM_MASK)
    #define mSPI_SS1_M_HSIOM_POS        (mSPI_ss1_m__0__HSIOM_SHIFT)
    #define mSPI_SS1_M_HSIOM_SEL_GPIO   (mSPI_ss1_m__0__HSIOM_GPIO)
    #define mSPI_SS1_M_HSIOM_SEL_SPI    (mSPI_ss1_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_SS1_PIN) */

#if (mSPI_SPI_MASTER_SS2_PIN)
    #define mSPI_SS2_M_HSIOM_REG    (*(reg32 *) mSPI_ss2_m__0__HSIOM)
    #define mSPI_SS2_M_HSIOM_PTR    ( (reg32 *) mSPI_ss2_m__0__HSIOM)
    
    #define mSPI_SS2_M_HSIOM_MASK       (mSPI_ss2_m__0__HSIOM_MASK)
    #define mSPI_SS2_M_HSIOM_POS        (mSPI_ss2_m__0__HSIOM_SHIFT)
    #define mSPI_SS2_M_HSIOM_SEL_GPIO   (mSPI_ss2_m__0__HSIOM_GPIO)
    #define mSPI_SS2_M_HSIOM_SEL_SPI    (mSPI_ss2_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_SS2_PIN) */

#if (mSPI_SPI_MASTER_SS3_PIN)
    #define mSPI_SS3_M_HSIOM_REG    (*(reg32 *) mSPI_ss3_m__0__HSIOM)
    #define mSPI_SS3_M_HSIOM_PTR    ( (reg32 *) mSPI_ss3_m__0__HSIOM)
    
    #define mSPI_SS3_M_HSIOM_MASK      (mSPI_ss3_m__0__HSIOM_MASK)
    #define mSPI_SS3_M_HSIOM_POS       (mSPI_ss3_m__0__HSIOM_SHIFT)
    #define mSPI_SS3_M_HSIOM_SEL_GPIO  (mSPI_ss3_m__0__HSIOM_GPIO)
    #define mSPI_SS3_M_HSIOM_SEL_SPI   (mSPI_ss3_m__0__HSIOM_SPI)
#endif /* (mSPI_SPI_MASTER_SS3_PIN) */

#if (mSPI_UART_RX_PIN)
    #define mSPI_RX_HSIOM_REG   (*(reg32 *) mSPI_rx__0__HSIOM)
    #define mSPI_RX_HSIOM_PTR   ( (reg32 *) mSPI_rx__0__HSIOM)
    
    #define mSPI_RX_HSIOM_MASK      (mSPI_rx__0__HSIOM_MASK)
    #define mSPI_RX_HSIOM_POS       (mSPI_rx__0__HSIOM_SHIFT)
    #define mSPI_RX_HSIOM_SEL_GPIO  (mSPI_rx__0__HSIOM_GPIO)
    #define mSPI_RX_HSIOM_SEL_UART  (mSPI_rx__0__HSIOM_UART)
#endif /* (mSPI_UART_RX_PIN) */

#if (mSPI_UART_RX_WAKE_PIN)
    #define mSPI_RX_WAKE_HSIOM_REG   (*(reg32 *) mSPI_rx_wake__0__HSIOM)
    #define mSPI_RX_WAKE_HSIOM_PTR   ( (reg32 *) mSPI_rx_wake__0__HSIOM)
    
    #define mSPI_RX_WAKE_HSIOM_MASK      (mSPI_rx_wake__0__HSIOM_MASK)
    #define mSPI_RX_WAKE_HSIOM_POS       (mSPI_rx_wake__0__HSIOM_SHIFT)
    #define mSPI_RX_WAKE_HSIOM_SEL_GPIO  (mSPI_rx_wake__0__HSIOM_GPIO)
    #define mSPI_RX_WAKE_HSIOM_SEL_UART  (mSPI_rx_wake__0__HSIOM_UART)
#endif /* (mSPI_UART_WAKE_RX_PIN) */

#if (mSPI_UART_CTS_PIN)
    #define mSPI_CTS_HSIOM_REG   (*(reg32 *) mSPI_cts__0__HSIOM)
    #define mSPI_CTS_HSIOM_PTR   ( (reg32 *) mSPI_cts__0__HSIOM)
    
    #define mSPI_CTS_HSIOM_MASK      (mSPI_cts__0__HSIOM_MASK)
    #define mSPI_CTS_HSIOM_POS       (mSPI_cts__0__HSIOM_SHIFT)
    #define mSPI_CTS_HSIOM_SEL_GPIO  (mSPI_cts__0__HSIOM_GPIO)
    #define mSPI_CTS_HSIOM_SEL_UART  (mSPI_cts__0__HSIOM_UART)
#endif /* (mSPI_UART_CTS_PIN) */

#if (mSPI_UART_TX_PIN)
    #define mSPI_TX_HSIOM_REG   (*(reg32 *) mSPI_tx__0__HSIOM)
    #define mSPI_TX_HSIOM_PTR   ( (reg32 *) mSPI_tx__0__HSIOM)
    
    #define mSPI_TX_HSIOM_MASK      (mSPI_tx__0__HSIOM_MASK)
    #define mSPI_TX_HSIOM_POS       (mSPI_tx__0__HSIOM_SHIFT)
    #define mSPI_TX_HSIOM_SEL_GPIO  (mSPI_tx__0__HSIOM_GPIO)
    #define mSPI_TX_HSIOM_SEL_UART  (mSPI_tx__0__HSIOM_UART)
#endif /* (mSPI_UART_TX_PIN) */

#if (mSPI_UART_RX_TX_PIN)
    #define mSPI_RX_TX_HSIOM_REG   (*(reg32 *) mSPI_rx_tx__0__HSIOM)
    #define mSPI_RX_TX_HSIOM_PTR   ( (reg32 *) mSPI_rx_tx__0__HSIOM)
    
    #define mSPI_RX_TX_HSIOM_MASK      (mSPI_rx_tx__0__HSIOM_MASK)
    #define mSPI_RX_TX_HSIOM_POS       (mSPI_rx_tx__0__HSIOM_SHIFT)
    #define mSPI_RX_TX_HSIOM_SEL_GPIO  (mSPI_rx_tx__0__HSIOM_GPIO)
    #define mSPI_RX_TX_HSIOM_SEL_UART  (mSPI_rx_tx__0__HSIOM_UART)
#endif /* (mSPI_UART_RX_TX_PIN) */

#if (mSPI_UART_RTS_PIN)
    #define mSPI_RTS_HSIOM_REG      (*(reg32 *) mSPI_rts__0__HSIOM)
    #define mSPI_RTS_HSIOM_PTR      ( (reg32 *) mSPI_rts__0__HSIOM)
    
    #define mSPI_RTS_HSIOM_MASK     (mSPI_rts__0__HSIOM_MASK)
    #define mSPI_RTS_HSIOM_POS      (mSPI_rts__0__HSIOM_SHIFT)    
    #define mSPI_RTS_HSIOM_SEL_GPIO (mSPI_rts__0__HSIOM_GPIO)
    #define mSPI_RTS_HSIOM_SEL_UART (mSPI_rts__0__HSIOM_UART)    
#endif /* (mSPI_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define mSPI_HSIOM_DEF_SEL      (0x00u)
#define mSPI_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for mSPI_CY_SCBIP_V0 
* and mSPI_CY_SCBIP_V1. It is not recommended to use them for 
* mSPI_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define mSPI_HSIOM_UART_SEL     (0x09u)
#define mSPI_HSIOM_I2C_SEL      (0x0Eu)
#define mSPI_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define mSPI_RX_SCL_MOSI_PIN_INDEX       (0u)
#define mSPI_TX_SDA_MISO_PIN_INDEX       (1u)
#define mSPI_CTS_SCLK_PIN_INDEX       (2u)
#define mSPI_RTS_SS0_PIN_INDEX       (3u)
#define mSPI_SS1_PIN_INDEX                  (4u)
#define mSPI_SS2_PIN_INDEX                  (5u)
#define mSPI_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define mSPI_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define mSPI_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << mSPI_RX_SCL_MOSI_PIN_INDEX)
#define mSPI_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << mSPI_TX_SDA_MISO_PIN_INDEX)
#define mSPI_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << mSPI_CTS_SCLK_PIN_INDEX)
#define mSPI_RTS_SS0_PIN_MASK     ((uint32) 0x01u << mSPI_RTS_SS0_PIN_INDEX)
#define mSPI_SS1_PIN_MASK                ((uint32) 0x01u << mSPI_SS1_PIN_INDEX)
#define mSPI_SS2_PIN_MASK                ((uint32) 0x01u << mSPI_SS2_PIN_INDEX)
#define mSPI_SS3_PIN_MASK                ((uint32) 0x01u << mSPI_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define mSPI_INTCFG_TYPE_MASK           (0x03u)
#define mSPI_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define mSPI_PIN_DM_ALG_HIZ  (0u)
#define mSPI_PIN_DM_DIG_HIZ  (1u)
#define mSPI_PIN_DM_OD_LO    (4u)
#define mSPI_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define mSPI_DM_MASK    (0x7u)
#define mSPI_DM_SIZE    (3u)
#define mSPI_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) mSPI_DM_MASK << (mSPI_DM_SIZE * (pos)))) >> \
                                                              (mSPI_DM_SIZE * (pos)) )

#if (mSPI_TX_SDA_MISO_PIN)
    #define mSPI_CHECK_TX_SDA_MISO_PIN_USED \
                (mSPI_PIN_DM_ALG_HIZ != \
                    mSPI_GET_P4_PIN_DM(mSPI_uart_tx_i2c_sda_spi_miso_PC, \
                                                   mSPI_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (mSPI_TX_SDA_MISO_PIN) */

#if (mSPI_RTS_SS0_PIN)
    #define mSPI_CHECK_RTS_SS0_PIN_USED \
                (mSPI_PIN_DM_ALG_HIZ != \
                    mSPI_GET_P4_PIN_DM(mSPI_uart_rts_spi_ss0_PC, \
                                                   mSPI_uart_rts_spi_ss0_SHIFT))
#endif /* (mSPI_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define mSPI_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define mSPI_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define mSPI_SET_HSIOM_SEL(reg, mask, pos, sel) mSPI_SET_REGISTER_BITS(reg, mask, pos, sel)
#define mSPI_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        mSPI_SET_REGISTER_BITS(reg, mask, pos, intType)
#define mSPI_SET_INP_DIS(reg, mask, val) mSPI_SET_REGISTER_BIT(reg, mask, val)

/* mSPI_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  mSPI_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (mSPI_CY_SCBIP_V0)
#if (mSPI_I2C_PINS)
    #define mSPI_SET_I2C_SCL_DR(val) mSPI_scl_Write(val)

    #define mSPI_SET_I2C_SCL_HSIOM_SEL(sel) \
                          mSPI_SET_HSIOM_SEL(mSPI_SCL_HSIOM_REG,  \
                                                         mSPI_SCL_HSIOM_MASK, \
                                                         mSPI_SCL_HSIOM_POS,  \
                                                         (sel))
    #define mSPI_WAIT_SCL_SET_HIGH  (0u == mSPI_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
    #define mSPI_SET_I2C_SCL_DR(val) \
                            mSPI_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define mSPI_SET_I2C_SCL_HSIOM_SEL(sel) \
                    mSPI_SET_HSIOM_SEL(mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   mSPI_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   mSPI_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define mSPI_WAIT_SCL_SET_HIGH  (0u == mSPI_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (mSPI_RX_SCL_MOSI_PIN)
    #define mSPI_SET_I2C_SCL_DR(val) \
                            mSPI_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define mSPI_SET_I2C_SCL_HSIOM_SEL(sel) \
                            mSPI_SET_HSIOM_SEL(mSPI_RX_SCL_MOSI_HSIOM_REG,  \
                                                           mSPI_RX_SCL_MOSI_HSIOM_MASK, \
                                                           mSPI_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define mSPI_WAIT_SCL_SET_HIGH  (0u == mSPI_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define mSPI_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define mSPI_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define mSPI_WAIT_SCL_SET_HIGH  (0u)
#endif /* (mSPI_I2C_PINS) */

/* SCB I2C: sda signal */
#if (mSPI_I2C_PINS)
    #define mSPI_WAIT_SDA_SET_HIGH  (0u == mSPI_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (mSPI_TX_SDA_MISO_PIN)
    #define mSPI_WAIT_SDA_SET_HIGH  (0u == mSPI_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define mSPI_WAIT_SDA_SET_HIGH  (0u)
#endif /* (mSPI_MOSI_SCL_RX_PIN) */
#endif /* (mSPI_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (mSPI_RX_SCL_MOSI_PIN)
    #define mSPI_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (mSPI_RX_WAKE_SCL_MOSI_PIN)
    #define mSPI_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) mSPI_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(mSPI_UART_RX_WAKE_PIN)
    #define mSPI_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) mSPI_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (mSPI_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define mSPI_REMOVE_MOSI_SCL_RX_WAKE_PIN    mSPI_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define mSPI_REMOVE_MOSI_SCL_RX_PIN         mSPI_REMOVE_RX_SCL_MOSI_PIN
#define mSPI_REMOVE_MISO_SDA_TX_PIN         mSPI_REMOVE_TX_SDA_MISO_PIN
#ifndef mSPI_REMOVE_SCLK_PIN
#define mSPI_REMOVE_SCLK_PIN                mSPI_REMOVE_CTS_SCLK_PIN
#endif /* mSPI_REMOVE_SCLK_PIN */
#ifndef mSPI_REMOVE_SS0_PIN
#define mSPI_REMOVE_SS0_PIN                 mSPI_REMOVE_RTS_SS0_PIN
#endif /* mSPI_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define mSPI_MOSI_SCL_RX_WAKE_PIN   mSPI_RX_WAKE_SCL_MOSI_PIN
#define mSPI_MOSI_SCL_RX_PIN        mSPI_RX_SCL_MOSI_PIN
#define mSPI_MISO_SDA_TX_PIN        mSPI_TX_SDA_MISO_PIN
#ifndef mSPI_SCLK_PIN
#define mSPI_SCLK_PIN               mSPI_CTS_SCLK_PIN
#endif /* mSPI_SCLK_PIN */
#ifndef mSPI_SS0_PIN
#define mSPI_SS0_PIN                mSPI_RTS_SS0_PIN
#endif /* mSPI_SS0_PIN */

#if (mSPI_MOSI_SCL_RX_WAKE_PIN)
    #define mSPI_MOSI_SCL_RX_WAKE_HSIOM_REG     mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_WAKE_HSIOM_PTR     mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_WAKE_HSIOM_MASK    mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_WAKE_HSIOM_POS     mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define mSPI_MOSI_SCL_RX_WAKE_INTCFG_REG    mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_WAKE_INTCFG_PTR    mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define mSPI_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  mSPI_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (mSPI_RX_WAKE_SCL_MOSI_PIN) */

#if (mSPI_MOSI_SCL_RX_PIN)
    #define mSPI_MOSI_SCL_RX_HSIOM_REG      mSPI_RX_SCL_MOSI_HSIOM_REG
    #define mSPI_MOSI_SCL_RX_HSIOM_PTR      mSPI_RX_SCL_MOSI_HSIOM_PTR
    #define mSPI_MOSI_SCL_RX_HSIOM_MASK     mSPI_RX_SCL_MOSI_HSIOM_MASK
    #define mSPI_MOSI_SCL_RX_HSIOM_POS      mSPI_RX_SCL_MOSI_HSIOM_POS
#endif /* (mSPI_MOSI_SCL_RX_PIN) */

#if (mSPI_MISO_SDA_TX_PIN)
    #define mSPI_MISO_SDA_TX_HSIOM_REG      mSPI_TX_SDA_MISO_HSIOM_REG
    #define mSPI_MISO_SDA_TX_HSIOM_PTR      mSPI_TX_SDA_MISO_HSIOM_REG
    #define mSPI_MISO_SDA_TX_HSIOM_MASK     mSPI_TX_SDA_MISO_HSIOM_REG
    #define mSPI_MISO_SDA_TX_HSIOM_POS      mSPI_TX_SDA_MISO_HSIOM_REG
#endif /* (mSPI_MISO_SDA_TX_PIN_PIN) */

#if (mSPI_SCLK_PIN)
    #ifndef mSPI_SCLK_HSIOM_REG
    #define mSPI_SCLK_HSIOM_REG     mSPI_CTS_SCLK_HSIOM_REG
    #define mSPI_SCLK_HSIOM_PTR     mSPI_CTS_SCLK_HSIOM_PTR
    #define mSPI_SCLK_HSIOM_MASK    mSPI_CTS_SCLK_HSIOM_MASK
    #define mSPI_SCLK_HSIOM_POS     mSPI_CTS_SCLK_HSIOM_POS
    #endif /* mSPI_SCLK_HSIOM_REG */
#endif /* (mSPI_SCLK_PIN) */

#if (mSPI_SS0_PIN)
    #ifndef mSPI_SS0_HSIOM_REG
    #define mSPI_SS0_HSIOM_REG      mSPI_RTS_SS0_HSIOM_REG
    #define mSPI_SS0_HSIOM_PTR      mSPI_RTS_SS0_HSIOM_PTR
    #define mSPI_SS0_HSIOM_MASK     mSPI_RTS_SS0_HSIOM_MASK
    #define mSPI_SS0_HSIOM_POS      mSPI_RTS_SS0_HSIOM_POS
    #endif /* mSPI_SS0_HSIOM_REG */
#endif /* (mSPI_SS0_PIN) */

#define mSPI_MOSI_SCL_RX_WAKE_PIN_INDEX mSPI_RX_WAKE_SCL_MOSI_PIN_INDEX
#define mSPI_MOSI_SCL_RX_PIN_INDEX      mSPI_RX_SCL_MOSI_PIN_INDEX
#define mSPI_MISO_SDA_TX_PIN_INDEX      mSPI_TX_SDA_MISO_PIN_INDEX
#ifndef mSPI_SCLK_PIN_INDEX
#define mSPI_SCLK_PIN_INDEX             mSPI_CTS_SCLK_PIN_INDEX
#endif /* mSPI_SCLK_PIN_INDEX */
#ifndef mSPI_SS0_PIN_INDEX
#define mSPI_SS0_PIN_INDEX              mSPI_RTS_SS0_PIN_INDEX
#endif /* mSPI_SS0_PIN_INDEX */

#define mSPI_MOSI_SCL_RX_WAKE_PIN_MASK mSPI_RX_WAKE_SCL_MOSI_PIN_MASK
#define mSPI_MOSI_SCL_RX_PIN_MASK      mSPI_RX_SCL_MOSI_PIN_MASK
#define mSPI_MISO_SDA_TX_PIN_MASK      mSPI_TX_SDA_MISO_PIN_MASK
#ifndef mSPI_SCLK_PIN_MASK
#define mSPI_SCLK_PIN_MASK             mSPI_CTS_SCLK_PIN_MASK
#endif /* mSPI_SCLK_PIN_MASK */
#ifndef mSPI_SS0_PIN_MASK
#define mSPI_SS0_PIN_MASK              mSPI_RTS_SS0_PIN_MASK
#endif /* mSPI_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_mSPI_H) */


/* [] END OF FILE */
