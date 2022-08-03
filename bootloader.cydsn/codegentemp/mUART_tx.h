/*******************************************************************************
* File Name: mUART_tx.h  
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

#if !defined(CY_PINS_mUART_tx_H) /* Pins mUART_tx_H */
#define CY_PINS_mUART_tx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "mUART_tx_aliases.h"


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
} mUART_tx_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   mUART_tx_Read(void);
void    mUART_tx_Write(uint8 value);
uint8   mUART_tx_ReadDataReg(void);
#if defined(mUART_tx__PC) || (CY_PSOC4_4200L) 
    void    mUART_tx_SetDriveMode(uint8 mode);
#endif
void    mUART_tx_SetInterruptMode(uint16 position, uint16 mode);
uint8   mUART_tx_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void mUART_tx_Sleep(void); 
void mUART_tx_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(mUART_tx__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define mUART_tx_DRIVE_MODE_BITS        (3)
    #define mUART_tx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - mUART_tx_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the mUART_tx_SetDriveMode() function.
         *  @{
         */
        #define mUART_tx_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define mUART_tx_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define mUART_tx_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define mUART_tx_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define mUART_tx_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define mUART_tx_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define mUART_tx_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define mUART_tx_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define mUART_tx_MASK               mUART_tx__MASK
#define mUART_tx_SHIFT              mUART_tx__SHIFT
#define mUART_tx_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in mUART_tx_SetInterruptMode() function.
     *  @{
     */
        #define mUART_tx_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define mUART_tx_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define mUART_tx_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define mUART_tx_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(mUART_tx__SIO)
    #define mUART_tx_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(mUART_tx__PC) && (CY_PSOC4_4200L)
    #define mUART_tx_USBIO_ENABLE               ((uint32)0x80000000u)
    #define mUART_tx_USBIO_DISABLE              ((uint32)(~mUART_tx_USBIO_ENABLE))
    #define mUART_tx_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define mUART_tx_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define mUART_tx_USBIO_ENTER_SLEEP          ((uint32)((1u << mUART_tx_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << mUART_tx_USBIO_SUSPEND_DEL_SHIFT)))
    #define mUART_tx_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << mUART_tx_USBIO_SUSPEND_SHIFT)))
    #define mUART_tx_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << mUART_tx_USBIO_SUSPEND_DEL_SHIFT)))
    #define mUART_tx_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(mUART_tx__PC)
    /* Port Configuration */
    #define mUART_tx_PC                 (* (reg32 *) mUART_tx__PC)
#endif
/* Pin State */
#define mUART_tx_PS                     (* (reg32 *) mUART_tx__PS)
/* Data Register */
#define mUART_tx_DR                     (* (reg32 *) mUART_tx__DR)
/* Input Buffer Disable Override */
#define mUART_tx_INP_DIS                (* (reg32 *) mUART_tx__PC2)

/* Interrupt configuration Registers */
#define mUART_tx_INTCFG                 (* (reg32 *) mUART_tx__INTCFG)
#define mUART_tx_INTSTAT                (* (reg32 *) mUART_tx__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define mUART_tx_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(mUART_tx__SIO)
    #define mUART_tx_SIO_REG            (* (reg32 *) mUART_tx__SIO)
#endif /* (mUART_tx__SIO_CFG) */

/* USBIO registers */
#if !defined(mUART_tx__PC) && (CY_PSOC4_4200L)
    #define mUART_tx_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define mUART_tx_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define mUART_tx_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define mUART_tx_DRIVE_MODE_SHIFT       (0x00u)
#define mUART_tx_DRIVE_MODE_MASK        (0x07u << mUART_tx_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins mUART_tx_H */


/* [] END OF FILE */
