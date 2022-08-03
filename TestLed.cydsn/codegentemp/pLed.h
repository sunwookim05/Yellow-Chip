/*******************************************************************************
* File Name: pLed.h  
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

#if !defined(CY_PINS_pLed_H) /* Pins pLed_H */
#define CY_PINS_pLed_H

#include "cytypes.h"
#include "cyfitter.h"
#include "pLed_aliases.h"


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
} pLed_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   pLed_Read(void);
void    pLed_Write(uint8 value);
uint8   pLed_ReadDataReg(void);
#if defined(pLed__PC) || (CY_PSOC4_4200L) 
    void    pLed_SetDriveMode(uint8 mode);
#endif
void    pLed_SetInterruptMode(uint16 position, uint16 mode);
uint8   pLed_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void pLed_Sleep(void); 
void pLed_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(pLed__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define pLed_DRIVE_MODE_BITS        (3)
    #define pLed_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - pLed_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the pLed_SetDriveMode() function.
         *  @{
         */
        #define pLed_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define pLed_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define pLed_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define pLed_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define pLed_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define pLed_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define pLed_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define pLed_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define pLed_MASK               pLed__MASK
#define pLed_SHIFT              pLed__SHIFT
#define pLed_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in pLed_SetInterruptMode() function.
     *  @{
     */
        #define pLed_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define pLed_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define pLed_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define pLed_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(pLed__SIO)
    #define pLed_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(pLed__PC) && (CY_PSOC4_4200L)
    #define pLed_USBIO_ENABLE               ((uint32)0x80000000u)
    #define pLed_USBIO_DISABLE              ((uint32)(~pLed_USBIO_ENABLE))
    #define pLed_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define pLed_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define pLed_USBIO_ENTER_SLEEP          ((uint32)((1u << pLed_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << pLed_USBIO_SUSPEND_DEL_SHIFT)))
    #define pLed_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << pLed_USBIO_SUSPEND_SHIFT)))
    #define pLed_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << pLed_USBIO_SUSPEND_DEL_SHIFT)))
    #define pLed_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(pLed__PC)
    /* Port Configuration */
    #define pLed_PC                 (* (reg32 *) pLed__PC)
#endif
/* Pin State */
#define pLed_PS                     (* (reg32 *) pLed__PS)
/* Data Register */
#define pLed_DR                     (* (reg32 *) pLed__DR)
/* Input Buffer Disable Override */
#define pLed_INP_DIS                (* (reg32 *) pLed__PC2)

/* Interrupt configuration Registers */
#define pLed_INTCFG                 (* (reg32 *) pLed__INTCFG)
#define pLed_INTSTAT                (* (reg32 *) pLed__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define pLed_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(pLed__SIO)
    #define pLed_SIO_REG            (* (reg32 *) pLed__SIO)
#endif /* (pLed__SIO_CFG) */

/* USBIO registers */
#if !defined(pLed__PC) && (CY_PSOC4_4200L)
    #define pLed_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define pLed_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define pLed_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define pLed_DRIVE_MODE_SHIFT       (0x00u)
#define pLed_DRIVE_MODE_MASK        (0x07u << pLed_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins pLed_H */


/* [] END OF FILE */
