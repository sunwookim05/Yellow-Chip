/*******************************************************************************
* File Name: cyfitter.h
* 
* PSoC Creator  4.4
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#ifndef INCLUDED_CYFITTER_H
#define INCLUDED_CYFITTER_H
#include "cydevice_trm.h"

/* C12M */
#define C12M__CTRL_REGISTER CYREG_PERI_PCLK_CTL7
#define C12M__DIV_ID 0x00000041u
#define C12M__DIV_REGISTER CYREG_PERI_DIV_16_CTL1
#define C12M__PA_DIV_ID 0x000000FFu

/* mPWM */
#define mPWM_cy_m0s8_tcpwm_1__CC CYREG_TCPWM_CNT4_CC
#define mPWM_cy_m0s8_tcpwm_1__CC_BUFF CYREG_TCPWM_CNT4_CC_BUFF
#define mPWM_cy_m0s8_tcpwm_1__COUNTER CYREG_TCPWM_CNT4_COUNTER
#define mPWM_cy_m0s8_tcpwm_1__CTRL CYREG_TCPWM_CNT4_CTRL
#define mPWM_cy_m0s8_tcpwm_1__INTR CYREG_TCPWM_CNT4_INTR
#define mPWM_cy_m0s8_tcpwm_1__INTR_MASK CYREG_TCPWM_CNT4_INTR_MASK
#define mPWM_cy_m0s8_tcpwm_1__INTR_MASKED CYREG_TCPWM_CNT4_INTR_MASKED
#define mPWM_cy_m0s8_tcpwm_1__INTR_SET CYREG_TCPWM_CNT4_INTR_SET
#define mPWM_cy_m0s8_tcpwm_1__PERIOD CYREG_TCPWM_CNT4_PERIOD
#define mPWM_cy_m0s8_tcpwm_1__PERIOD_BUFF CYREG_TCPWM_CNT4_PERIOD_BUFF
#define mPWM_cy_m0s8_tcpwm_1__STATUS CYREG_TCPWM_CNT4_STATUS
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMD CYREG_TCPWM_CMD
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_MASK 0x10u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_SHIFT 4u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_MASK 0x1000u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_SHIFT 12u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_MASK 0x10000000u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_SHIFT 28u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_MASK 0x100000u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_SHIFT 20u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL CYREG_TCPWM_CTRL
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK 0x10u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL_SHIFT 4u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE CYREG_TCPWM_INTR_CAUSE
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_MASK 0x10u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_SHIFT 4u
#define mPWM_cy_m0s8_tcpwm_1__TCPWM_NUMBER 4u
#define mPWM_cy_m0s8_tcpwm_1__TR_CTRL0 CYREG_TCPWM_CNT4_TR_CTRL0
#define mPWM_cy_m0s8_tcpwm_1__TR_CTRL1 CYREG_TCPWM_CNT4_TR_CTRL1
#define mPWM_cy_m0s8_tcpwm_1__TR_CTRL2 CYREG_TCPWM_CNT4_TR_CTRL2

/* mUART */
#define mUART_rx__0__DR CYREG_GPIO_PRT4_DR
#define mUART_rx__0__DR_CLR CYREG_GPIO_PRT4_DR_CLR
#define mUART_rx__0__DR_INV CYREG_GPIO_PRT4_DR_INV
#define mUART_rx__0__DR_SET CYREG_GPIO_PRT4_DR_SET
#define mUART_rx__0__HSIOM CYREG_HSIOM_PORT_SEL4
#define mUART_rx__0__HSIOM_GPIO 0u
#define mUART_rx__0__HSIOM_I2C 14u
#define mUART_rx__0__HSIOM_I2C_SCL 14u
#define mUART_rx__0__HSIOM_MASK 0x0000000Fu
#define mUART_rx__0__HSIOM_SHIFT 0u
#define mUART_rx__0__HSIOM_SPI 15u
#define mUART_rx__0__HSIOM_SPI_MOSI 15u
#define mUART_rx__0__HSIOM_UART 9u
#define mUART_rx__0__HSIOM_UART_RX 9u
#define mUART_rx__0__INTCFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_rx__0__INTR CYREG_GPIO_PRT4_INTR
#define mUART_rx__0__INTR_CFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_rx__0__INTSTAT CYREG_GPIO_PRT4_INTR
#define mUART_rx__0__MASK 0x01u
#define mUART_rx__0__PC CYREG_GPIO_PRT4_PC
#define mUART_rx__0__PC2 CYREG_GPIO_PRT4_PC2
#define mUART_rx__0__PORT 4u
#define mUART_rx__0__PS CYREG_GPIO_PRT4_PS
#define mUART_rx__0__SHIFT 0u
#define mUART_rx__DR CYREG_GPIO_PRT4_DR
#define mUART_rx__DR_CLR CYREG_GPIO_PRT4_DR_CLR
#define mUART_rx__DR_INV CYREG_GPIO_PRT4_DR_INV
#define mUART_rx__DR_SET CYREG_GPIO_PRT4_DR_SET
#define mUART_rx__INTCFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_rx__INTR CYREG_GPIO_PRT4_INTR
#define mUART_rx__INTR_CFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_rx__INTSTAT CYREG_GPIO_PRT4_INTR
#define mUART_rx__MASK 0x01u
#define mUART_rx__PC CYREG_GPIO_PRT4_PC
#define mUART_rx__PC2 CYREG_GPIO_PRT4_PC2
#define mUART_rx__PORT 4u
#define mUART_rx__PS CYREG_GPIO_PRT4_PS
#define mUART_rx__SHIFT 0u
#define mUART_SCB__CTRL CYREG_SCB0_CTRL
#define mUART_SCB__EZ_DATA0 CYREG_SCB0_EZ_DATA0
#define mUART_SCB__EZ_DATA1 CYREG_SCB0_EZ_DATA1
#define mUART_SCB__EZ_DATA10 CYREG_SCB0_EZ_DATA10
#define mUART_SCB__EZ_DATA11 CYREG_SCB0_EZ_DATA11
#define mUART_SCB__EZ_DATA12 CYREG_SCB0_EZ_DATA12
#define mUART_SCB__EZ_DATA13 CYREG_SCB0_EZ_DATA13
#define mUART_SCB__EZ_DATA14 CYREG_SCB0_EZ_DATA14
#define mUART_SCB__EZ_DATA15 CYREG_SCB0_EZ_DATA15
#define mUART_SCB__EZ_DATA16 CYREG_SCB0_EZ_DATA16
#define mUART_SCB__EZ_DATA17 CYREG_SCB0_EZ_DATA17
#define mUART_SCB__EZ_DATA18 CYREG_SCB0_EZ_DATA18
#define mUART_SCB__EZ_DATA19 CYREG_SCB0_EZ_DATA19
#define mUART_SCB__EZ_DATA2 CYREG_SCB0_EZ_DATA2
#define mUART_SCB__EZ_DATA20 CYREG_SCB0_EZ_DATA20
#define mUART_SCB__EZ_DATA21 CYREG_SCB0_EZ_DATA21
#define mUART_SCB__EZ_DATA22 CYREG_SCB0_EZ_DATA22
#define mUART_SCB__EZ_DATA23 CYREG_SCB0_EZ_DATA23
#define mUART_SCB__EZ_DATA24 CYREG_SCB0_EZ_DATA24
#define mUART_SCB__EZ_DATA25 CYREG_SCB0_EZ_DATA25
#define mUART_SCB__EZ_DATA26 CYREG_SCB0_EZ_DATA26
#define mUART_SCB__EZ_DATA27 CYREG_SCB0_EZ_DATA27
#define mUART_SCB__EZ_DATA28 CYREG_SCB0_EZ_DATA28
#define mUART_SCB__EZ_DATA29 CYREG_SCB0_EZ_DATA29
#define mUART_SCB__EZ_DATA3 CYREG_SCB0_EZ_DATA3
#define mUART_SCB__EZ_DATA30 CYREG_SCB0_EZ_DATA30
#define mUART_SCB__EZ_DATA31 CYREG_SCB0_EZ_DATA31
#define mUART_SCB__EZ_DATA4 CYREG_SCB0_EZ_DATA4
#define mUART_SCB__EZ_DATA5 CYREG_SCB0_EZ_DATA5
#define mUART_SCB__EZ_DATA6 CYREG_SCB0_EZ_DATA6
#define mUART_SCB__EZ_DATA7 CYREG_SCB0_EZ_DATA7
#define mUART_SCB__EZ_DATA8 CYREG_SCB0_EZ_DATA8
#define mUART_SCB__EZ_DATA9 CYREG_SCB0_EZ_DATA9
#define mUART_SCB__I2C_CFG CYREG_SCB0_I2C_CFG
#define mUART_SCB__I2C_CTRL CYREG_SCB0_I2C_CTRL
#define mUART_SCB__I2C_M_CMD CYREG_SCB0_I2C_M_CMD
#define mUART_SCB__I2C_S_CMD CYREG_SCB0_I2C_S_CMD
#define mUART_SCB__I2C_STATUS CYREG_SCB0_I2C_STATUS
#define mUART_SCB__INTR_CAUSE CYREG_SCB0_INTR_CAUSE
#define mUART_SCB__INTR_I2C_EC CYREG_SCB0_INTR_I2C_EC
#define mUART_SCB__INTR_I2C_EC_MASK CYREG_SCB0_INTR_I2C_EC_MASK
#define mUART_SCB__INTR_I2C_EC_MASKED CYREG_SCB0_INTR_I2C_EC_MASKED
#define mUART_SCB__INTR_M CYREG_SCB0_INTR_M
#define mUART_SCB__INTR_M_MASK CYREG_SCB0_INTR_M_MASK
#define mUART_SCB__INTR_M_MASKED CYREG_SCB0_INTR_M_MASKED
#define mUART_SCB__INTR_M_SET CYREG_SCB0_INTR_M_SET
#define mUART_SCB__INTR_RX CYREG_SCB0_INTR_RX
#define mUART_SCB__INTR_RX_MASK CYREG_SCB0_INTR_RX_MASK
#define mUART_SCB__INTR_RX_MASKED CYREG_SCB0_INTR_RX_MASKED
#define mUART_SCB__INTR_RX_SET CYREG_SCB0_INTR_RX_SET
#define mUART_SCB__INTR_S CYREG_SCB0_INTR_S
#define mUART_SCB__INTR_S_MASK CYREG_SCB0_INTR_S_MASK
#define mUART_SCB__INTR_S_MASKED CYREG_SCB0_INTR_S_MASKED
#define mUART_SCB__INTR_S_SET CYREG_SCB0_INTR_S_SET
#define mUART_SCB__INTR_SPI_EC CYREG_SCB0_INTR_SPI_EC
#define mUART_SCB__INTR_SPI_EC_MASK CYREG_SCB0_INTR_SPI_EC_MASK
#define mUART_SCB__INTR_SPI_EC_MASKED CYREG_SCB0_INTR_SPI_EC_MASKED
#define mUART_SCB__INTR_TX CYREG_SCB0_INTR_TX
#define mUART_SCB__INTR_TX_MASK CYREG_SCB0_INTR_TX_MASK
#define mUART_SCB__INTR_TX_MASKED CYREG_SCB0_INTR_TX_MASKED
#define mUART_SCB__INTR_TX_SET CYREG_SCB0_INTR_TX_SET
#define mUART_SCB__RX_CTRL CYREG_SCB0_RX_CTRL
#define mUART_SCB__RX_FIFO_CTRL CYREG_SCB0_RX_FIFO_CTRL
#define mUART_SCB__RX_FIFO_RD CYREG_SCB0_RX_FIFO_RD
#define mUART_SCB__RX_FIFO_RD_SILENT CYREG_SCB0_RX_FIFO_RD_SILENT
#define mUART_SCB__RX_FIFO_STATUS CYREG_SCB0_RX_FIFO_STATUS
#define mUART_SCB__RX_MATCH CYREG_SCB0_RX_MATCH
#define mUART_SCB__SPI_CTRL CYREG_SCB0_SPI_CTRL
#define mUART_SCB__SPI_STATUS CYREG_SCB0_SPI_STATUS
#define mUART_SCB__SS0_POSISTION 0u
#define mUART_SCB__SS1_POSISTION 1u
#define mUART_SCB__SS2_POSISTION 2u
#define mUART_SCB__SS3_POSISTION 3u
#define mUART_SCB__STATUS CYREG_SCB0_STATUS
#define mUART_SCB__TX_CTRL CYREG_SCB0_TX_CTRL
#define mUART_SCB__TX_FIFO_CTRL CYREG_SCB0_TX_FIFO_CTRL
#define mUART_SCB__TX_FIFO_STATUS CYREG_SCB0_TX_FIFO_STATUS
#define mUART_SCB__TX_FIFO_WR CYREG_SCB0_TX_FIFO_WR
#define mUART_SCB__UART_CTRL CYREG_SCB0_UART_CTRL
#define mUART_SCB__UART_FLOW_CTRL CYREG_SCB0_UART_FLOW_CTRL
#define mUART_SCB__UART_RX_CTRL CYREG_SCB0_UART_RX_CTRL
#define mUART_SCB__UART_RX_STATUS CYREG_SCB0_UART_RX_STATUS
#define mUART_SCB__UART_TX_CTRL CYREG_SCB0_UART_TX_CTRL
#define mUART_SCB_IRQ__INTC_CLR_EN_REG CYREG_CM0P_ICER
#define mUART_SCB_IRQ__INTC_CLR_PD_REG CYREG_CM0P_ICPR
#define mUART_SCB_IRQ__INTC_MASK 0x80u
#define mUART_SCB_IRQ__INTC_NUMBER 7u
#define mUART_SCB_IRQ__INTC_PRIOR_MASK 0xC0000000u
#define mUART_SCB_IRQ__INTC_PRIOR_NUM 3u
#define mUART_SCB_IRQ__INTC_PRIOR_REG CYREG_CM0P_IPR1
#define mUART_SCB_IRQ__INTC_SET_EN_REG CYREG_CM0P_ISER
#define mUART_SCB_IRQ__INTC_SET_PD_REG CYREG_CM0P_ISPR
#define mUART_SCBCLK__CTRL_REGISTER CYREG_PERI_PCLK_CTL0
#define mUART_SCBCLK__DIV_ID 0x00000040u
#define mUART_SCBCLK__DIV_REGISTER CYREG_PERI_DIV_16_CTL0
#define mUART_SCBCLK__PA_DIV_ID 0x000000FFu
#define mUART_tx__0__DR CYREG_GPIO_PRT4_DR
#define mUART_tx__0__DR_CLR CYREG_GPIO_PRT4_DR_CLR
#define mUART_tx__0__DR_INV CYREG_GPIO_PRT4_DR_INV
#define mUART_tx__0__DR_SET CYREG_GPIO_PRT4_DR_SET
#define mUART_tx__0__HSIOM CYREG_HSIOM_PORT_SEL4
#define mUART_tx__0__HSIOM_GPIO 0u
#define mUART_tx__0__HSIOM_I2C 14u
#define mUART_tx__0__HSIOM_I2C_SDA 14u
#define mUART_tx__0__HSIOM_MASK 0x000000F0u
#define mUART_tx__0__HSIOM_SHIFT 4u
#define mUART_tx__0__HSIOM_SPI 15u
#define mUART_tx__0__HSIOM_SPI_MISO 15u
#define mUART_tx__0__HSIOM_UART 9u
#define mUART_tx__0__HSIOM_UART_TX 9u
#define mUART_tx__0__INTCFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_tx__0__INTR CYREG_GPIO_PRT4_INTR
#define mUART_tx__0__INTR_CFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_tx__0__INTSTAT CYREG_GPIO_PRT4_INTR
#define mUART_tx__0__MASK 0x02u
#define mUART_tx__0__PC CYREG_GPIO_PRT4_PC
#define mUART_tx__0__PC2 CYREG_GPIO_PRT4_PC2
#define mUART_tx__0__PORT 4u
#define mUART_tx__0__PS CYREG_GPIO_PRT4_PS
#define mUART_tx__0__SHIFT 1u
#define mUART_tx__DR CYREG_GPIO_PRT4_DR
#define mUART_tx__DR_CLR CYREG_GPIO_PRT4_DR_CLR
#define mUART_tx__DR_INV CYREG_GPIO_PRT4_DR_INV
#define mUART_tx__DR_SET CYREG_GPIO_PRT4_DR_SET
#define mUART_tx__INTCFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_tx__INTR CYREG_GPIO_PRT4_INTR
#define mUART_tx__INTR_CFG CYREG_GPIO_PRT4_INTR_CFG
#define mUART_tx__INTSTAT CYREG_GPIO_PRT4_INTR
#define mUART_tx__MASK 0x02u
#define mUART_tx__PC CYREG_GPIO_PRT4_PC
#define mUART_tx__PC2 CYREG_GPIO_PRT4_PC2
#define mUART_tx__PORT 4u
#define mUART_tx__PS CYREG_GPIO_PRT4_PS
#define mUART_tx__SHIFT 1u

/* BUZZER */
#define BUZZER__0__DR CYREG_GPIO_PRT2_DR
#define BUZZER__0__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define BUZZER__0__DR_INV CYREG_GPIO_PRT2_DR_INV
#define BUZZER__0__DR_SET CYREG_GPIO_PRT2_DR_SET
#define BUZZER__0__HSIOM CYREG_HSIOM_PORT_SEL2
#define BUZZER__0__HSIOM_MASK 0x0000000Fu
#define BUZZER__0__HSIOM_SHIFT 0u
#define BUZZER__0__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define BUZZER__0__INTR CYREG_GPIO_PRT2_INTR
#define BUZZER__0__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define BUZZER__0__INTSTAT CYREG_GPIO_PRT2_INTR
#define BUZZER__0__MASK 0x01u
#define BUZZER__0__PC CYREG_GPIO_PRT2_PC
#define BUZZER__0__PC2 CYREG_GPIO_PRT2_PC2
#define BUZZER__0__PORT 2u
#define BUZZER__0__PS CYREG_GPIO_PRT2_PS
#define BUZZER__0__SHIFT 0u
#define BUZZER__DR CYREG_GPIO_PRT2_DR
#define BUZZER__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define BUZZER__DR_INV CYREG_GPIO_PRT2_DR_INV
#define BUZZER__DR_SET CYREG_GPIO_PRT2_DR_SET
#define BUZZER__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define BUZZER__INTR CYREG_GPIO_PRT2_INTR
#define BUZZER__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define BUZZER__INTSTAT CYREG_GPIO_PRT2_INTR
#define BUZZER__MASK 0x01u
#define BUZZER__PC CYREG_GPIO_PRT2_PC
#define BUZZER__PC2 CYREG_GPIO_PRT2_PC2
#define BUZZER__PORT 2u
#define BUZZER__PS CYREG_GPIO_PRT2_PS
#define BUZZER__SHIFT 0u

/* Miscellaneous */
#define CY_PROJECT_NAME "BUZZER"
#define CY_VERSION "PSoC Creator  4.4"
#define CYDEV_BANDGAP_VOLTAGE 1.200
#define CYDEV_BCLK__HFCLK__HZ 24000000U
#define CYDEV_BCLK__HFCLK__KHZ 24000U
#define CYDEV_BCLK__HFCLK__MHZ 24U
#define CYDEV_BCLK__SYSCLK__HZ 24000000U
#define CYDEV_BCLK__SYSCLK__KHZ 24000U
#define CYDEV_BCLK__SYSCLK__MHZ 24U
#define CYDEV_CHIP_DIE_LEOPARD 1u
#define CYDEV_CHIP_DIE_PSOC4A 26u
#define CYDEV_CHIP_DIE_PSOC5LP 2u
#define CYDEV_CHIP_DIE_PSOC5TM 3u
#define CYDEV_CHIP_DIE_TMA4 4u
#define CYDEV_CHIP_DIE_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_FM0P 5u
#define CYDEV_CHIP_FAMILY_FM3 6u
#define CYDEV_CHIP_FAMILY_FM4 7u
#define CYDEV_CHIP_FAMILY_PSOC3 1u
#define CYDEV_CHIP_FAMILY_PSOC4 2u
#define CYDEV_CHIP_FAMILY_PSOC5 3u
#define CYDEV_CHIP_FAMILY_PSOC6 4u
#define CYDEV_CHIP_FAMILY_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_USED CYDEV_CHIP_FAMILY_PSOC4
#define CYDEV_CHIP_JTAG_ID 0x190A11A9u
#define CYDEV_CHIP_MEMBER_3A 1u
#define CYDEV_CHIP_MEMBER_4A 26u
#define CYDEV_CHIP_MEMBER_4AA 25u
#define CYDEV_CHIP_MEMBER_4AB 30u
#define CYDEV_CHIP_MEMBER_4AC 14u
#define CYDEV_CHIP_MEMBER_4AD 15u
#define CYDEV_CHIP_MEMBER_4AE 16u
#define CYDEV_CHIP_MEMBER_4D 20u
#define CYDEV_CHIP_MEMBER_4E 6u
#define CYDEV_CHIP_MEMBER_4F 27u
#define CYDEV_CHIP_MEMBER_4G 4u
#define CYDEV_CHIP_MEMBER_4H 24u
#define CYDEV_CHIP_MEMBER_4I 32u
#define CYDEV_CHIP_MEMBER_4J 21u
#define CYDEV_CHIP_MEMBER_4K 22u
#define CYDEV_CHIP_MEMBER_4L 31u
#define CYDEV_CHIP_MEMBER_4M 29u
#define CYDEV_CHIP_MEMBER_4N 11u
#define CYDEV_CHIP_MEMBER_4O 8u
#define CYDEV_CHIP_MEMBER_4P 28u
#define CYDEV_CHIP_MEMBER_4Q 17u
#define CYDEV_CHIP_MEMBER_4R 9u
#define CYDEV_CHIP_MEMBER_4S 12u
#define CYDEV_CHIP_MEMBER_4T 10u
#define CYDEV_CHIP_MEMBER_4U 5u
#define CYDEV_CHIP_MEMBER_4V 23u
#define CYDEV_CHIP_MEMBER_4W 13u
#define CYDEV_CHIP_MEMBER_4X 7u
#define CYDEV_CHIP_MEMBER_4Y 18u
#define CYDEV_CHIP_MEMBER_4Z 19u
#define CYDEV_CHIP_MEMBER_5A 3u
#define CYDEV_CHIP_MEMBER_5B 2u
#define CYDEV_CHIP_MEMBER_6A 33u
#define CYDEV_CHIP_MEMBER_FM3 37u
#define CYDEV_CHIP_MEMBER_FM4 38u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 34u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 35u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 36u
#define CYDEV_CHIP_MEMBER_UNKNOWN 0u
#define CYDEV_CHIP_MEMBER_USED CYDEV_CHIP_MEMBER_4J
#define CYDEV_CHIP_DIE_EXPECT CYDEV_CHIP_MEMBER_USED
#define CYDEV_CHIP_DIE_ACTUAL CYDEV_CHIP_DIE_EXPECT
#define CYDEV_CHIP_REV_LEOPARD_ES1 0u
#define CYDEV_CHIP_REV_LEOPARD_ES2 1u
#define CYDEV_CHIP_REV_LEOPARD_ES3 3u
#define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3u
#define CYDEV_CHIP_REV_PSOC4A_ES0 17u
#define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17u
#define CYDEV_CHIP_REV_PSOC5LP_ES0 0u
#define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES0 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES1 1u
#define CYDEV_CHIP_REV_PSOC5TM_PRODUCTION 1u
#define CYDEV_CHIP_REV_TMA4_ES 17u
#define CYDEV_CHIP_REV_TMA4_ES2 33u
#define CYDEV_CHIP_REV_TMA4_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_3A_ES1 0u
#define CYDEV_CHIP_REVISION_3A_ES2 1u
#define CYDEV_CHIP_REVISION_3A_ES3 3u
#define CYDEV_CHIP_REVISION_3A_PRODUCTION 3u
#define CYDEV_CHIP_REVISION_4A_ES0 17u
#define CYDEV_CHIP_REVISION_4A_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4AA_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AB_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AC_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AD_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AE_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4D_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD 0u
#define CYDEV_CHIP_REVISION_4E_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0u
#define CYDEV_CHIP_REVISION_4G_ES 17u
#define CYDEV_CHIP_REVISION_4G_ES2 33u
#define CYDEV_CHIP_REVISION_4G_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4H_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4I_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4J_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4K_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4L_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4M_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4N_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4O_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4P_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Q_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4R_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4S_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4T_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4U_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4V_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4W_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4X_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Y_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Z_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_5A_ES0 0u
#define CYDEV_CHIP_REVISION_5A_ES1 1u
#define CYDEV_CHIP_REVISION_5A_PRODUCTION 1u
#define CYDEV_CHIP_REVISION_5B_ES0 0u
#define CYDEV_CHIP_REVISION_5B_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_6A_ES 17u
#define CYDEV_CHIP_REVISION_6A_NO_UDB 33u
#define CYDEV_CHIP_REVISION_6A_PRODUCTION 33u
#define CYDEV_CHIP_REVISION_FM3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_FM4_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_USED CYDEV_CHIP_REVISION_4J_PRODUCTION
#define CYDEV_CHIP_REV_EXPECT CYDEV_CHIP_REVISION_USED
#define CYDEV_CONFIG_READ_ACCELERATOR 1
#define CYDEV_CONFIG_UNUSED_IO_AllowButWarn 0
#define CYDEV_CONFIG_UNUSED_IO_AllowWithInfo 1
#define CYDEV_CONFIG_UNUSED_IO_Disallowed 2
#define CYDEV_CONFIG_UNUSED_IO CYDEV_CONFIG_UNUSED_IO_Disallowed
#define CYDEV_CONFIGURATION_COMPRESSED 1
#define CYDEV_CONFIGURATION_MODE_COMPRESSED 0
#define CYDEV_CONFIGURATION_MODE CYDEV_CONFIGURATION_MODE_COMPRESSED
#define CYDEV_CONFIGURATION_MODE_DMA 2
#define CYDEV_CONFIGURATION_MODE_UNCOMPRESSED 1
#define CYDEV_DEBUG_PROTECT_KILL 4
#define CYDEV_DEBUG_PROTECT_OPEN 1
#define CYDEV_DEBUG_PROTECT CYDEV_DEBUG_PROTECT_OPEN
#define CYDEV_DEBUG_PROTECT_PROTECTED 2
#define CYDEV_DEBUGGING_DPS_Disable 3
#define CYDEV_DEBUGGING_DPS_SWD 2
#define CYDEV_DEBUGGING_DPS CYDEV_DEBUGGING_DPS_SWD
#define CYDEV_DEBUGGING_ENABLE 1
#define CYDEV_DFT_SELECT_CLK0 8u
#define CYDEV_DFT_SELECT_CLK1 9u
#define CYDEV_HEAP_SIZE 0x80
#define CYDEV_IMO_TRIMMED_BY_USB 0u
#define CYDEV_IMO_TRIMMED_BY_WCO 0u
#define CYDEV_IS_EXPORTING_CODE 0
#define CYDEV_IS_IMPORTING_CODE 0
#define CYDEV_PROJ_TYPE 2
#define CYDEV_PROJ_TYPE_BOOTLOADER 1
#define CYDEV_PROJ_TYPE_LAUNCHER 5
#define CYDEV_PROJ_TYPE_LOADABLE 2
#define CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER 4
#define CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER 3
#define CYDEV_PROJ_TYPE_STANDARD 0
#define CYDEV_STACK_SIZE 0x0400
#define CYDEV_USE_BUNDLED_CMSIS 1
#define CYDEV_VARIABLE_VDDA 1
#define CYDEV_VDD 3.3
#define CYDEV_VDD_MV 3300
#define CYDEV_WDT_GENERATE_ISR 0u
#define CYIPBLOCK_m0s8cpussv3_VERSION 1
#define CYIPBLOCK_m0s8csdv2_VERSION 1
#define CYIPBLOCK_m0s8ioss_VERSION 1
#define CYIPBLOCK_m0s8lcd_VERSION 2
#define CYIPBLOCK_m0s8lpcomp_VERSION 2
#define CYIPBLOCK_m0s8peri_VERSION 1
#define CYIPBLOCK_m0s8scb_VERSION 2
#define CYIPBLOCK_m0s8tcpwm_VERSION 2
#define CYIPBLOCK_m0s8wco_VERSION 1
#define CYIPBLOCK_s8srsslt_VERSION 1
#define CYDEV_BOOTLOADER_ENABLE 0

#endif /* INCLUDED_CYFITTER_H */
