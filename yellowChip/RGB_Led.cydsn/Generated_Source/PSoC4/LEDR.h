/*******************************************************************************
* File Name: LEDR.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_LEDR_H) /* Pins LEDR_H */
#define CY_PINS_LEDR_H

#include "cytypes.h"
#include "cyfitter.h"
#include "LEDR_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} LEDR_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   LEDR_Read(void);
void    LEDR_Write(uint8 value);
uint8   LEDR_ReadDataReg(void);
#if defined(LEDR__PC) || (CY_PSOC4_4200L) 
    void    LEDR_SetDriveMode(uint8 mode);
#endif
void    LEDR_SetInterruptMode(uint16 position, uint16 mode);
uint8   LEDR_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void LEDR_Sleep(void); 
void LEDR_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(LEDR__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define LEDR_DRIVE_MODE_BITS        (3)
    #define LEDR_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - LEDR_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the LEDR_SetDriveMode() function.
         *  @{
         */
        #define LEDR_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define LEDR_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define LEDR_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define LEDR_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define LEDR_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define LEDR_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define LEDR_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define LEDR_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define LEDR_MASK               LEDR__MASK
#define LEDR_SHIFT              LEDR__SHIFT
#define LEDR_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in LEDR_SetInterruptMode() function.
     *  @{
     */
        #define LEDR_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define LEDR_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define LEDR_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define LEDR_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(LEDR__SIO)
    #define LEDR_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(LEDR__PC) && (CY_PSOC4_4200L)
    #define LEDR_USBIO_ENABLE               ((uint32)0x80000000u)
    #define LEDR_USBIO_DISABLE              ((uint32)(~LEDR_USBIO_ENABLE))
    #define LEDR_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define LEDR_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define LEDR_USBIO_ENTER_SLEEP          ((uint32)((1u << LEDR_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << LEDR_USBIO_SUSPEND_DEL_SHIFT)))
    #define LEDR_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << LEDR_USBIO_SUSPEND_SHIFT)))
    #define LEDR_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << LEDR_USBIO_SUSPEND_DEL_SHIFT)))
    #define LEDR_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(LEDR__PC)
    /* Port Configuration */
    #define LEDR_PC                 (* (reg32 *) LEDR__PC)
#endif
/* Pin State */
#define LEDR_PS                     (* (reg32 *) LEDR__PS)
/* Data Register */
#define LEDR_DR                     (* (reg32 *) LEDR__DR)
/* Input Buffer Disable Override */
#define LEDR_INP_DIS                (* (reg32 *) LEDR__PC2)

/* Interrupt configuration Registers */
#define LEDR_INTCFG                 (* (reg32 *) LEDR__INTCFG)
#define LEDR_INTSTAT                (* (reg32 *) LEDR__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define LEDR_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(LEDR__SIO)
    #define LEDR_SIO_REG            (* (reg32 *) LEDR__SIO)
#endif /* (LEDR__SIO_CFG) */

/* USBIO registers */
#if !defined(LEDR__PC) && (CY_PSOC4_4200L)
    #define LEDR_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define LEDR_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define LEDR_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define LEDR_DRIVE_MODE_SHIFT       (0x00u)
#define LEDR_DRIVE_MODE_MASK        (0x07u << LEDR_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins LEDR_H */


/* [] END OF FILE */
