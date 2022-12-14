/*******************************************************************************
* File Name: cyfittergnu.inc
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

.ifndef INCLUDED_CYFITTERGNU_INC
.set INCLUDED_CYFITTERGNU_INC, 1
.include "cydevicegnu_trm.inc"

/* C12M */
.set C12M__CTRL_REGISTER, CYREG_PERI_PCLK_CTL7
.set C12M__DIV_ID, 0x00000040
.set C12M__DIV_REGISTER, CYREG_PERI_DIV_16_CTL0
.set C12M__PA_DIV_ID, 0x000000FF

/* LEDB */
.set LEDB__0__DR, CYREG_GPIO_PRT1_DR
.set LEDB__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set LEDB__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set LEDB__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set LEDB__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set LEDB__0__HSIOM_MASK, 0x00000F00
.set LEDB__0__HSIOM_SHIFT, 8
.set LEDB__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set LEDB__0__INTR, CYREG_GPIO_PRT1_INTR
.set LEDB__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set LEDB__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set LEDB__0__MASK, 0x04
.set LEDB__0__PC, CYREG_GPIO_PRT1_PC
.set LEDB__0__PC2, CYREG_GPIO_PRT1_PC2
.set LEDB__0__PORT, 1
.set LEDB__0__PS, CYREG_GPIO_PRT1_PS
.set LEDB__0__SHIFT, 2
.set LEDB__DR, CYREG_GPIO_PRT1_DR
.set LEDB__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set LEDB__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set LEDB__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set LEDB__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set LEDB__INTR, CYREG_GPIO_PRT1_INTR
.set LEDB__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set LEDB__INTSTAT, CYREG_GPIO_PRT1_INTR
.set LEDB__MASK, 0x04
.set LEDB__PC, CYREG_GPIO_PRT1_PC
.set LEDB__PC2, CYREG_GPIO_PRT1_PC2
.set LEDB__PORT, 1
.set LEDB__PS, CYREG_GPIO_PRT1_PS
.set LEDB__SHIFT, 2

/* LEDG */
.set LEDG__0__DR, CYREG_GPIO_PRT2_DR
.set LEDG__0__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set LEDG__0__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set LEDG__0__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set LEDG__0__HSIOM, CYREG_HSIOM_PORT_SEL2
.set LEDG__0__HSIOM_MASK, 0x0F000000
.set LEDG__0__HSIOM_SHIFT, 24
.set LEDG__0__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDG__0__INTR, CYREG_GPIO_PRT2_INTR
.set LEDG__0__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDG__0__INTSTAT, CYREG_GPIO_PRT2_INTR
.set LEDG__0__MASK, 0x40
.set LEDG__0__PC, CYREG_GPIO_PRT2_PC
.set LEDG__0__PC2, CYREG_GPIO_PRT2_PC2
.set LEDG__0__PORT, 2
.set LEDG__0__PS, CYREG_GPIO_PRT2_PS
.set LEDG__0__SHIFT, 6
.set LEDG__DR, CYREG_GPIO_PRT2_DR
.set LEDG__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set LEDG__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set LEDG__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set LEDG__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDG__INTR, CYREG_GPIO_PRT2_INTR
.set LEDG__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDG__INTSTAT, CYREG_GPIO_PRT2_INTR
.set LEDG__MASK, 0x40
.set LEDG__PC, CYREG_GPIO_PRT2_PC
.set LEDG__PC2, CYREG_GPIO_PRT2_PC2
.set LEDG__PORT, 2
.set LEDG__PS, CYREG_GPIO_PRT2_PS
.set LEDG__SHIFT, 6

/* LEDR */
.set LEDR__0__DR, CYREG_GPIO_PRT2_DR
.set LEDR__0__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set LEDR__0__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set LEDR__0__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set LEDR__0__HSIOM, CYREG_HSIOM_PORT_SEL2
.set LEDR__0__HSIOM_MASK, 0x0000000F
.set LEDR__0__HSIOM_SHIFT, 0
.set LEDR__0__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDR__0__INTR, CYREG_GPIO_PRT2_INTR
.set LEDR__0__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDR__0__INTSTAT, CYREG_GPIO_PRT2_INTR
.set LEDR__0__MASK, 0x01
.set LEDR__0__PC, CYREG_GPIO_PRT2_PC
.set LEDR__0__PC2, CYREG_GPIO_PRT2_PC2
.set LEDR__0__PORT, 2
.set LEDR__0__PS, CYREG_GPIO_PRT2_PS
.set LEDR__0__SHIFT, 0
.set LEDR__DR, CYREG_GPIO_PRT2_DR
.set LEDR__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set LEDR__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set LEDR__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set LEDR__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDR__INTR, CYREG_GPIO_PRT2_INTR
.set LEDR__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set LEDR__INTSTAT, CYREG_GPIO_PRT2_INTR
.set LEDR__MASK, 0x01
.set LEDR__PC, CYREG_GPIO_PRT2_PC
.set LEDR__PC2, CYREG_GPIO_PRT2_PC2
.set LEDR__PORT, 2
.set LEDR__PS, CYREG_GPIO_PRT2_PS
.set LEDR__SHIFT, 0

/* mPWM0 */
.set mPWM0_cy_m0s8_tcpwm_1__CC, CYREG_TCPWM_CNT4_CC
.set mPWM0_cy_m0s8_tcpwm_1__CC_BUFF, CYREG_TCPWM_CNT4_CC_BUFF
.set mPWM0_cy_m0s8_tcpwm_1__COUNTER, CYREG_TCPWM_CNT4_COUNTER
.set mPWM0_cy_m0s8_tcpwm_1__CTRL, CYREG_TCPWM_CNT4_CTRL
.set mPWM0_cy_m0s8_tcpwm_1__INTR, CYREG_TCPWM_CNT4_INTR
.set mPWM0_cy_m0s8_tcpwm_1__INTR_MASK, CYREG_TCPWM_CNT4_INTR_MASK
.set mPWM0_cy_m0s8_tcpwm_1__INTR_MASKED, CYREG_TCPWM_CNT4_INTR_MASKED
.set mPWM0_cy_m0s8_tcpwm_1__INTR_SET, CYREG_TCPWM_CNT4_INTR_SET
.set mPWM0_cy_m0s8_tcpwm_1__PERIOD, CYREG_TCPWM_CNT4_PERIOD
.set mPWM0_cy_m0s8_tcpwm_1__PERIOD_BUFF, CYREG_TCPWM_CNT4_PERIOD_BUFF
.set mPWM0_cy_m0s8_tcpwm_1__STATUS, CYREG_TCPWM_CNT4_STATUS
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMD, CYREG_TCPWM_CMD
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_MASK, 0x10
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_SHIFT, 4
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_MASK, 0x1000
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_SHIFT, 12
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_MASK, 0x10000000
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_SHIFT, 28
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_MASK, 0x100000
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_SHIFT, 20
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL, CYREG_TCPWM_CTRL
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK, 0x10
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_CTRL_SHIFT, 4
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE, CYREG_TCPWM_INTR_CAUSE
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_MASK, 0x10
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_SHIFT, 4
.set mPWM0_cy_m0s8_tcpwm_1__TCPWM_NUMBER, 4
.set mPWM0_cy_m0s8_tcpwm_1__TR_CTRL0, CYREG_TCPWM_CNT4_TR_CTRL0
.set mPWM0_cy_m0s8_tcpwm_1__TR_CTRL1, CYREG_TCPWM_CNT4_TR_CTRL1
.set mPWM0_cy_m0s8_tcpwm_1__TR_CTRL2, CYREG_TCPWM_CNT4_TR_CTRL2

/* mPWM1 */
.set mPWM1_cy_m0s8_tcpwm_1__CC, CYREG_TCPWM_CNT1_CC
.set mPWM1_cy_m0s8_tcpwm_1__CC_BUFF, CYREG_TCPWM_CNT1_CC_BUFF
.set mPWM1_cy_m0s8_tcpwm_1__COUNTER, CYREG_TCPWM_CNT1_COUNTER
.set mPWM1_cy_m0s8_tcpwm_1__CTRL, CYREG_TCPWM_CNT1_CTRL
.set mPWM1_cy_m0s8_tcpwm_1__INTR, CYREG_TCPWM_CNT1_INTR
.set mPWM1_cy_m0s8_tcpwm_1__INTR_MASK, CYREG_TCPWM_CNT1_INTR_MASK
.set mPWM1_cy_m0s8_tcpwm_1__INTR_MASKED, CYREG_TCPWM_CNT1_INTR_MASKED
.set mPWM1_cy_m0s8_tcpwm_1__INTR_SET, CYREG_TCPWM_CNT1_INTR_SET
.set mPWM1_cy_m0s8_tcpwm_1__PERIOD, CYREG_TCPWM_CNT1_PERIOD
.set mPWM1_cy_m0s8_tcpwm_1__PERIOD_BUFF, CYREG_TCPWM_CNT1_PERIOD_BUFF
.set mPWM1_cy_m0s8_tcpwm_1__STATUS, CYREG_TCPWM_CNT1_STATUS
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMD, CYREG_TCPWM_CMD
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_MASK, 0x02
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_SHIFT, 1
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_MASK, 0x200
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_SHIFT, 9
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_MASK, 0x2000000
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_SHIFT, 25
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_MASK, 0x20000
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_SHIFT, 17
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CTRL, CYREG_TCPWM_CTRL
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK, 0x02
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_CTRL_SHIFT, 1
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE, CYREG_TCPWM_INTR_CAUSE
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_MASK, 0x02
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_SHIFT, 1
.set mPWM1_cy_m0s8_tcpwm_1__TCPWM_NUMBER, 1
.set mPWM1_cy_m0s8_tcpwm_1__TR_CTRL0, CYREG_TCPWM_CNT1_TR_CTRL0
.set mPWM1_cy_m0s8_tcpwm_1__TR_CTRL1, CYREG_TCPWM_CNT1_TR_CTRL1
.set mPWM1_cy_m0s8_tcpwm_1__TR_CTRL2, CYREG_TCPWM_CNT1_TR_CTRL2

/* mPWM2 */
.set mPWM2_cy_m0s8_tcpwm_1__CC, CYREG_TCPWM_CNT3_CC
.set mPWM2_cy_m0s8_tcpwm_1__CC_BUFF, CYREG_TCPWM_CNT3_CC_BUFF
.set mPWM2_cy_m0s8_tcpwm_1__COUNTER, CYREG_TCPWM_CNT3_COUNTER
.set mPWM2_cy_m0s8_tcpwm_1__CTRL, CYREG_TCPWM_CNT3_CTRL
.set mPWM2_cy_m0s8_tcpwm_1__INTR, CYREG_TCPWM_CNT3_INTR
.set mPWM2_cy_m0s8_tcpwm_1__INTR_MASK, CYREG_TCPWM_CNT3_INTR_MASK
.set mPWM2_cy_m0s8_tcpwm_1__INTR_MASKED, CYREG_TCPWM_CNT3_INTR_MASKED
.set mPWM2_cy_m0s8_tcpwm_1__INTR_SET, CYREG_TCPWM_CNT3_INTR_SET
.set mPWM2_cy_m0s8_tcpwm_1__PERIOD, CYREG_TCPWM_CNT3_PERIOD
.set mPWM2_cy_m0s8_tcpwm_1__PERIOD_BUFF, CYREG_TCPWM_CNT3_PERIOD_BUFF
.set mPWM2_cy_m0s8_tcpwm_1__STATUS, CYREG_TCPWM_CNT3_STATUS
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMD, CYREG_TCPWM_CMD
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_MASK, 0x08
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_SHIFT, 3
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_MASK, 0x800
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_SHIFT, 11
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_MASK, 0x8000000
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_SHIFT, 27
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_MASK, 0x80000
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_SHIFT, 19
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL, CYREG_TCPWM_CTRL
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK, 0x08
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_CTRL_SHIFT, 3
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE, CYREG_TCPWM_INTR_CAUSE
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_MASK, 0x08
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_SHIFT, 3
.set mPWM2_cy_m0s8_tcpwm_1__TCPWM_NUMBER, 3
.set mPWM2_cy_m0s8_tcpwm_1__TR_CTRL0, CYREG_TCPWM_CNT3_TR_CTRL0
.set mPWM2_cy_m0s8_tcpwm_1__TR_CTRL1, CYREG_TCPWM_CNT3_TR_CTRL1
.set mPWM2_cy_m0s8_tcpwm_1__TR_CTRL2, CYREG_TCPWM_CNT3_TR_CTRL2

/* mUART_rx */
.set mUART_rx__0__DR, CYREG_GPIO_PRT4_DR
.set mUART_rx__0__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set mUART_rx__0__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set mUART_rx__0__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set mUART_rx__0__HSIOM, CYREG_HSIOM_PORT_SEL4
.set mUART_rx__0__HSIOM_GPIO, 0
.set mUART_rx__0__HSIOM_I2C, 14
.set mUART_rx__0__HSIOM_I2C_SCL, 14
.set mUART_rx__0__HSIOM_MASK, 0x0000000F
.set mUART_rx__0__HSIOM_SHIFT, 0
.set mUART_rx__0__HSIOM_SPI, 15
.set mUART_rx__0__HSIOM_SPI_MOSI, 15
.set mUART_rx__0__HSIOM_UART, 9
.set mUART_rx__0__HSIOM_UART_RX, 9
.set mUART_rx__0__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_rx__0__INTR, CYREG_GPIO_PRT4_INTR
.set mUART_rx__0__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_rx__0__INTSTAT, CYREG_GPIO_PRT4_INTR
.set mUART_rx__0__MASK, 0x01
.set mUART_rx__0__PC, CYREG_GPIO_PRT4_PC
.set mUART_rx__0__PC2, CYREG_GPIO_PRT4_PC2
.set mUART_rx__0__PORT, 4
.set mUART_rx__0__PS, CYREG_GPIO_PRT4_PS
.set mUART_rx__0__SHIFT, 0
.set mUART_rx__DR, CYREG_GPIO_PRT4_DR
.set mUART_rx__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set mUART_rx__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set mUART_rx__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set mUART_rx__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_rx__INTR, CYREG_GPIO_PRT4_INTR
.set mUART_rx__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_rx__INTSTAT, CYREG_GPIO_PRT4_INTR
.set mUART_rx__MASK, 0x01
.set mUART_rx__PC, CYREG_GPIO_PRT4_PC
.set mUART_rx__PC2, CYREG_GPIO_PRT4_PC2
.set mUART_rx__PORT, 4
.set mUART_rx__PS, CYREG_GPIO_PRT4_PS
.set mUART_rx__SHIFT, 0

/* mUART_SCB */
.set mUART_SCB__CTRL, CYREG_SCB0_CTRL
.set mUART_SCB__EZ_DATA0, CYREG_SCB0_EZ_DATA0
.set mUART_SCB__EZ_DATA1, CYREG_SCB0_EZ_DATA1
.set mUART_SCB__EZ_DATA10, CYREG_SCB0_EZ_DATA10
.set mUART_SCB__EZ_DATA11, CYREG_SCB0_EZ_DATA11
.set mUART_SCB__EZ_DATA12, CYREG_SCB0_EZ_DATA12
.set mUART_SCB__EZ_DATA13, CYREG_SCB0_EZ_DATA13
.set mUART_SCB__EZ_DATA14, CYREG_SCB0_EZ_DATA14
.set mUART_SCB__EZ_DATA15, CYREG_SCB0_EZ_DATA15
.set mUART_SCB__EZ_DATA16, CYREG_SCB0_EZ_DATA16
.set mUART_SCB__EZ_DATA17, CYREG_SCB0_EZ_DATA17
.set mUART_SCB__EZ_DATA18, CYREG_SCB0_EZ_DATA18
.set mUART_SCB__EZ_DATA19, CYREG_SCB0_EZ_DATA19
.set mUART_SCB__EZ_DATA2, CYREG_SCB0_EZ_DATA2
.set mUART_SCB__EZ_DATA20, CYREG_SCB0_EZ_DATA20
.set mUART_SCB__EZ_DATA21, CYREG_SCB0_EZ_DATA21
.set mUART_SCB__EZ_DATA22, CYREG_SCB0_EZ_DATA22
.set mUART_SCB__EZ_DATA23, CYREG_SCB0_EZ_DATA23
.set mUART_SCB__EZ_DATA24, CYREG_SCB0_EZ_DATA24
.set mUART_SCB__EZ_DATA25, CYREG_SCB0_EZ_DATA25
.set mUART_SCB__EZ_DATA26, CYREG_SCB0_EZ_DATA26
.set mUART_SCB__EZ_DATA27, CYREG_SCB0_EZ_DATA27
.set mUART_SCB__EZ_DATA28, CYREG_SCB0_EZ_DATA28
.set mUART_SCB__EZ_DATA29, CYREG_SCB0_EZ_DATA29
.set mUART_SCB__EZ_DATA3, CYREG_SCB0_EZ_DATA3
.set mUART_SCB__EZ_DATA30, CYREG_SCB0_EZ_DATA30
.set mUART_SCB__EZ_DATA31, CYREG_SCB0_EZ_DATA31
.set mUART_SCB__EZ_DATA4, CYREG_SCB0_EZ_DATA4
.set mUART_SCB__EZ_DATA5, CYREG_SCB0_EZ_DATA5
.set mUART_SCB__EZ_DATA6, CYREG_SCB0_EZ_DATA6
.set mUART_SCB__EZ_DATA7, CYREG_SCB0_EZ_DATA7
.set mUART_SCB__EZ_DATA8, CYREG_SCB0_EZ_DATA8
.set mUART_SCB__EZ_DATA9, CYREG_SCB0_EZ_DATA9
.set mUART_SCB__I2C_CFG, CYREG_SCB0_I2C_CFG
.set mUART_SCB__I2C_CTRL, CYREG_SCB0_I2C_CTRL
.set mUART_SCB__I2C_M_CMD, CYREG_SCB0_I2C_M_CMD
.set mUART_SCB__I2C_S_CMD, CYREG_SCB0_I2C_S_CMD
.set mUART_SCB__I2C_STATUS, CYREG_SCB0_I2C_STATUS
.set mUART_SCB__INTR_CAUSE, CYREG_SCB0_INTR_CAUSE
.set mUART_SCB__INTR_I2C_EC, CYREG_SCB0_INTR_I2C_EC
.set mUART_SCB__INTR_I2C_EC_MASK, CYREG_SCB0_INTR_I2C_EC_MASK
.set mUART_SCB__INTR_I2C_EC_MASKED, CYREG_SCB0_INTR_I2C_EC_MASKED
.set mUART_SCB__INTR_M, CYREG_SCB0_INTR_M
.set mUART_SCB__INTR_M_MASK, CYREG_SCB0_INTR_M_MASK
.set mUART_SCB__INTR_M_MASKED, CYREG_SCB0_INTR_M_MASKED
.set mUART_SCB__INTR_M_SET, CYREG_SCB0_INTR_M_SET
.set mUART_SCB__INTR_RX, CYREG_SCB0_INTR_RX
.set mUART_SCB__INTR_RX_MASK, CYREG_SCB0_INTR_RX_MASK
.set mUART_SCB__INTR_RX_MASKED, CYREG_SCB0_INTR_RX_MASKED
.set mUART_SCB__INTR_RX_SET, CYREG_SCB0_INTR_RX_SET
.set mUART_SCB__INTR_S, CYREG_SCB0_INTR_S
.set mUART_SCB__INTR_S_MASK, CYREG_SCB0_INTR_S_MASK
.set mUART_SCB__INTR_S_MASKED, CYREG_SCB0_INTR_S_MASKED
.set mUART_SCB__INTR_S_SET, CYREG_SCB0_INTR_S_SET
.set mUART_SCB__INTR_SPI_EC, CYREG_SCB0_INTR_SPI_EC
.set mUART_SCB__INTR_SPI_EC_MASK, CYREG_SCB0_INTR_SPI_EC_MASK
.set mUART_SCB__INTR_SPI_EC_MASKED, CYREG_SCB0_INTR_SPI_EC_MASKED
.set mUART_SCB__INTR_TX, CYREG_SCB0_INTR_TX
.set mUART_SCB__INTR_TX_MASK, CYREG_SCB0_INTR_TX_MASK
.set mUART_SCB__INTR_TX_MASKED, CYREG_SCB0_INTR_TX_MASKED
.set mUART_SCB__INTR_TX_SET, CYREG_SCB0_INTR_TX_SET
.set mUART_SCB__RX_CTRL, CYREG_SCB0_RX_CTRL
.set mUART_SCB__RX_FIFO_CTRL, CYREG_SCB0_RX_FIFO_CTRL
.set mUART_SCB__RX_FIFO_RD, CYREG_SCB0_RX_FIFO_RD
.set mUART_SCB__RX_FIFO_RD_SILENT, CYREG_SCB0_RX_FIFO_RD_SILENT
.set mUART_SCB__RX_FIFO_STATUS, CYREG_SCB0_RX_FIFO_STATUS
.set mUART_SCB__RX_MATCH, CYREG_SCB0_RX_MATCH
.set mUART_SCB__SPI_CTRL, CYREG_SCB0_SPI_CTRL
.set mUART_SCB__SPI_STATUS, CYREG_SCB0_SPI_STATUS
.set mUART_SCB__SS0_POSISTION, 0
.set mUART_SCB__SS1_POSISTION, 1
.set mUART_SCB__SS2_POSISTION, 2
.set mUART_SCB__SS3_POSISTION, 3
.set mUART_SCB__STATUS, CYREG_SCB0_STATUS
.set mUART_SCB__TX_CTRL, CYREG_SCB0_TX_CTRL
.set mUART_SCB__TX_FIFO_CTRL, CYREG_SCB0_TX_FIFO_CTRL
.set mUART_SCB__TX_FIFO_STATUS, CYREG_SCB0_TX_FIFO_STATUS
.set mUART_SCB__TX_FIFO_WR, CYREG_SCB0_TX_FIFO_WR
.set mUART_SCB__UART_CTRL, CYREG_SCB0_UART_CTRL
.set mUART_SCB__UART_FLOW_CTRL, CYREG_SCB0_UART_FLOW_CTRL
.set mUART_SCB__UART_RX_CTRL, CYREG_SCB0_UART_RX_CTRL
.set mUART_SCB__UART_RX_STATUS, CYREG_SCB0_UART_RX_STATUS
.set mUART_SCB__UART_TX_CTRL, CYREG_SCB0_UART_TX_CTRL

/* mUART_SCBCLK */
.set mUART_SCBCLK__CTRL_REGISTER, CYREG_PERI_PCLK_CTL0
.set mUART_SCBCLK__DIV_ID, 0x00000041
.set mUART_SCBCLK__DIV_REGISTER, CYREG_PERI_DIV_16_CTL1
.set mUART_SCBCLK__PA_DIV_ID, 0x000000FF

/* Miscellaneous */
.set mUART_tx__0__DR, CYREG_GPIO_PRT4_DR
.set mUART_tx__0__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set mUART_tx__0__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set mUART_tx__0__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set mUART_tx__0__HSIOM, CYREG_HSIOM_PORT_SEL4
.set mUART_tx__0__HSIOM_GPIO, 0
.set mUART_tx__0__HSIOM_I2C, 14
.set mUART_tx__0__HSIOM_I2C_SDA, 14
.set mUART_tx__0__HSIOM_MASK, 0x000000F0
.set mUART_tx__0__HSIOM_SHIFT, 4
.set mUART_tx__0__HSIOM_SPI, 15
.set mUART_tx__0__HSIOM_SPI_MISO, 15
.set mUART_tx__0__HSIOM_UART, 9
.set mUART_tx__0__HSIOM_UART_TX, 9
.set mUART_tx__0__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_tx__0__INTR, CYREG_GPIO_PRT4_INTR
.set mUART_tx__0__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_tx__0__INTSTAT, CYREG_GPIO_PRT4_INTR
.set mUART_tx__0__MASK, 0x02
.set mUART_tx__0__PC, CYREG_GPIO_PRT4_PC
.set mUART_tx__0__PC2, CYREG_GPIO_PRT4_PC2
.set mUART_tx__0__PORT, 4
.set mUART_tx__0__PS, CYREG_GPIO_PRT4_PS
.set mUART_tx__0__SHIFT, 1
.set mUART_tx__DR, CYREG_GPIO_PRT4_DR
.set mUART_tx__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set mUART_tx__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set mUART_tx__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set mUART_tx__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_tx__INTR, CYREG_GPIO_PRT4_INTR
.set mUART_tx__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set mUART_tx__INTSTAT, CYREG_GPIO_PRT4_INTR
.set mUART_tx__MASK, 0x02
.set mUART_tx__PC, CYREG_GPIO_PRT4_PC
.set mUART_tx__PC2, CYREG_GPIO_PRT4_PC2
.set mUART_tx__PORT, 4
.set mUART_tx__PS, CYREG_GPIO_PRT4_PS
.set mUART_tx__SHIFT, 1
.set CYDEV_BCLK__HFCLK__HZ, 24000000
.set CYDEV_BCLK__HFCLK__KHZ, 24000
.set CYDEV_BCLK__HFCLK__MHZ, 24
.set CYDEV_BCLK__SYSCLK__HZ, 24000000
.set CYDEV_BCLK__SYSCLK__KHZ, 24000
.set CYDEV_BCLK__SYSCLK__MHZ, 24
.set CYDEV_CHIP_DIE_LEOPARD, 1
.set CYDEV_CHIP_DIE_PSOC4A, 26
.set CYDEV_CHIP_DIE_PSOC5LP, 2
.set CYDEV_CHIP_DIE_PSOC5TM, 3
.set CYDEV_CHIP_DIE_TMA4, 4
.set CYDEV_CHIP_DIE_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_FM0P, 5
.set CYDEV_CHIP_FAMILY_FM3, 6
.set CYDEV_CHIP_FAMILY_FM4, 7
.set CYDEV_CHIP_FAMILY_PSOC3, 1
.set CYDEV_CHIP_FAMILY_PSOC4, 2
.set CYDEV_CHIP_FAMILY_PSOC5, 3
.set CYDEV_CHIP_FAMILY_PSOC6, 4
.set CYDEV_CHIP_FAMILY_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_USED, CYDEV_CHIP_FAMILY_PSOC4
.set CYDEV_CHIP_JTAG_ID, 0x190A11A9
.set CYDEV_CHIP_MEMBER_3A, 1
.set CYDEV_CHIP_MEMBER_4A, 26
.set CYDEV_CHIP_MEMBER_4AA, 25
.set CYDEV_CHIP_MEMBER_4AB, 30
.set CYDEV_CHIP_MEMBER_4AC, 14
.set CYDEV_CHIP_MEMBER_4AD, 15
.set CYDEV_CHIP_MEMBER_4AE, 16
.set CYDEV_CHIP_MEMBER_4D, 20
.set CYDEV_CHIP_MEMBER_4E, 6
.set CYDEV_CHIP_MEMBER_4F, 27
.set CYDEV_CHIP_MEMBER_4G, 4
.set CYDEV_CHIP_MEMBER_4H, 24
.set CYDEV_CHIP_MEMBER_4I, 32
.set CYDEV_CHIP_MEMBER_4J, 21
.set CYDEV_CHIP_MEMBER_4K, 22
.set CYDEV_CHIP_MEMBER_4L, 31
.set CYDEV_CHIP_MEMBER_4M, 29
.set CYDEV_CHIP_MEMBER_4N, 11
.set CYDEV_CHIP_MEMBER_4O, 8
.set CYDEV_CHIP_MEMBER_4P, 28
.set CYDEV_CHIP_MEMBER_4Q, 17
.set CYDEV_CHIP_MEMBER_4R, 9
.set CYDEV_CHIP_MEMBER_4S, 12
.set CYDEV_CHIP_MEMBER_4T, 10
.set CYDEV_CHIP_MEMBER_4U, 5
.set CYDEV_CHIP_MEMBER_4V, 23
.set CYDEV_CHIP_MEMBER_4W, 13
.set CYDEV_CHIP_MEMBER_4X, 7
.set CYDEV_CHIP_MEMBER_4Y, 18
.set CYDEV_CHIP_MEMBER_4Z, 19
.set CYDEV_CHIP_MEMBER_5A, 3
.set CYDEV_CHIP_MEMBER_5B, 2
.set CYDEV_CHIP_MEMBER_6A, 33
.set CYDEV_CHIP_MEMBER_FM3, 37
.set CYDEV_CHIP_MEMBER_FM4, 38
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1, 34
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2, 35
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3, 36
.set CYDEV_CHIP_MEMBER_UNKNOWN, 0
.set CYDEV_CHIP_MEMBER_USED, CYDEV_CHIP_MEMBER_4J
.set CYDEV_CHIP_DIE_EXPECT, CYDEV_CHIP_MEMBER_USED
.set CYDEV_CHIP_DIE_ACTUAL, CYDEV_CHIP_DIE_EXPECT
.set CYDEV_CHIP_REV_LEOPARD_ES1, 0
.set CYDEV_CHIP_REV_LEOPARD_ES2, 1
.set CYDEV_CHIP_REV_LEOPARD_ES3, 3
.set CYDEV_CHIP_REV_LEOPARD_PRODUCTION, 3
.set CYDEV_CHIP_REV_PSOC4A_ES0, 17
.set CYDEV_CHIP_REV_PSOC4A_PRODUCTION, 17
.set CYDEV_CHIP_REV_PSOC5LP_ES0, 0
.set CYDEV_CHIP_REV_PSOC5LP_PRODUCTION, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES0, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES1, 1
.set CYDEV_CHIP_REV_PSOC5TM_PRODUCTION, 1
.set CYDEV_CHIP_REV_TMA4_ES, 17
.set CYDEV_CHIP_REV_TMA4_ES2, 33
.set CYDEV_CHIP_REV_TMA4_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_3A_ES1, 0
.set CYDEV_CHIP_REVISION_3A_ES2, 1
.set CYDEV_CHIP_REVISION_3A_ES3, 3
.set CYDEV_CHIP_REVISION_3A_PRODUCTION, 3
.set CYDEV_CHIP_REVISION_4A_ES0, 17
.set CYDEV_CHIP_REVISION_4A_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4AA_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AB_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AC_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AD_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AE_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4D_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD, 0
.set CYDEV_CHIP_REVISION_4E_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256K, 0
.set CYDEV_CHIP_REVISION_4G_ES, 17
.set CYDEV_CHIP_REVISION_4G_ES2, 33
.set CYDEV_CHIP_REVISION_4G_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4H_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4I_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4J_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4K_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4L_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4M_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4N_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4O_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4P_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Q_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4R_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4S_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4T_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4U_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4V_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4W_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4X_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Y_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Z_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_5A_ES0, 0
.set CYDEV_CHIP_REVISION_5A_ES1, 1
.set CYDEV_CHIP_REVISION_5A_PRODUCTION, 1
.set CYDEV_CHIP_REVISION_5B_ES0, 0
.set CYDEV_CHIP_REVISION_5B_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_6A_ES, 17
.set CYDEV_CHIP_REVISION_6A_NO_UDB, 33
.set CYDEV_CHIP_REVISION_6A_PRODUCTION, 33
.set CYDEV_CHIP_REVISION_FM3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_FM4_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_USED, CYDEV_CHIP_REVISION_4J_PRODUCTION
.set CYDEV_CHIP_REV_EXPECT, CYDEV_CHIP_REVISION_USED
.set CYDEV_CONFIG_READ_ACCELERATOR, 1
.set CYDEV_CONFIG_UNUSED_IO_AllowButWarn, 0
.set CYDEV_CONFIG_UNUSED_IO_AllowWithInfo, 1
.set CYDEV_CONFIG_UNUSED_IO_Disallowed, 2
.set CYDEV_CONFIG_UNUSED_IO, CYDEV_CONFIG_UNUSED_IO_Disallowed
.set CYDEV_CONFIGURATION_COMPRESSED, 1
.set CYDEV_CONFIGURATION_MODE_COMPRESSED, 0
.set CYDEV_CONFIGURATION_MODE, CYDEV_CONFIGURATION_MODE_COMPRESSED
.set CYDEV_CONFIGURATION_MODE_DMA, 2
.set CYDEV_CONFIGURATION_MODE_UNCOMPRESSED, 1
.set CYDEV_DEBUG_PROTECT_KILL, 4
.set CYDEV_DEBUG_PROTECT_OPEN, 1
.set CYDEV_DEBUG_PROTECT, CYDEV_DEBUG_PROTECT_OPEN
.set CYDEV_DEBUG_PROTECT_PROTECTED, 2
.set CYDEV_DEBUGGING_DPS_Disable, 3
.set CYDEV_DEBUGGING_DPS_SWD, 2
.set CYDEV_DEBUGGING_DPS, CYDEV_DEBUGGING_DPS_SWD
.set CYDEV_DEBUGGING_ENABLE, 1
.set CYDEV_DFT_SELECT_CLK0, 8
.set CYDEV_DFT_SELECT_CLK1, 9
.set CYDEV_HEAP_SIZE, 0x400
.set CYDEV_IMO_TRIMMED_BY_USB, 0
.set CYDEV_IMO_TRIMMED_BY_WCO, 0
.set CYDEV_IS_EXPORTING_CODE, 0
.set CYDEV_IS_IMPORTING_CODE, 0
.set CYDEV_PROJ_TYPE, 2
.set CYDEV_PROJ_TYPE_BOOTLOADER, 1
.set CYDEV_PROJ_TYPE_LAUNCHER, 5
.set CYDEV_PROJ_TYPE_LOADABLE, 2
.set CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER, 4
.set CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER, 3
.set CYDEV_PROJ_TYPE_STANDARD, 0
.set CYDEV_STACK_SIZE, 0x0400
.set CYDEV_USE_BUNDLED_CMSIS, 1
.set CYDEV_VARIABLE_VDDA, 1
.set CYDEV_VDD, 5
.set CYDEV_VDD_MV, 5000
.set CYDEV_WDT_GENERATE_ISR, 0
.set CYIPBLOCK_m0s8cpussv3_VERSION, 1
.set CYIPBLOCK_m0s8csdv2_VERSION, 1
.set CYIPBLOCK_m0s8ioss_VERSION, 1
.set CYIPBLOCK_m0s8lcd_VERSION, 2
.set CYIPBLOCK_m0s8lpcomp_VERSION, 2
.set CYIPBLOCK_m0s8peri_VERSION, 1
.set CYIPBLOCK_m0s8scb_VERSION, 2
.set CYIPBLOCK_m0s8tcpwm_VERSION, 2
.set CYIPBLOCK_m0s8wco_VERSION, 1
.set CYIPBLOCK_s8srsslt_VERSION, 1
.set CYDEV_BOOTLOADER_ENABLE, 0
.endif
