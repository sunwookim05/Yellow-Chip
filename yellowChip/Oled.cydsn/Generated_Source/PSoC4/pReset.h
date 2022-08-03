/*******************************************************************************
* File Name: pReset.h  
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

#if !defined(CY_PINS_pReset_H) /* Pins pReset_H */
#define CY_PINS_pReset_H

#include "cytypes.h"
#include "cyfitter.h"
#include "pReset_aliases.h"


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
} pReset_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   pReset_Read(void);
void    pReset_Write(uint8 value);
uint8   pReset_ReadDataReg(void);
#if defined(pReset__PC) || (CY_PSOC4_4200L) 
    void    pReset_SetDriveMode(uint8 mode);
#endif
void    pReset_SetInterruptMode(uint16 position, uint16 mode);
uint8   pReset_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void pReset_Sleep(void); 
void pReset_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(pReset__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define pReset_DRIVE_MODE_BITS        (3)
    #define pReset_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - pReset_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the pReset_SetDriveMode() function.
         *  @{
         */
        #define pReset_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define pReset_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define pReset_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define pReset_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define pReset_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define pReset_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define pReset_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define pReset_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define pReset_MASK               pReset__MASK
#define pReset_SHIFT              pReset__SHIFT
#define pReset_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in pReset_SetInterruptMode() function.
     *  @{
     */
        #define pReset_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define pReset_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define pReset_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define pReset_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(pReset__SIO)
    #define pReset_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(pReset__PC) && (CY_PSOC4_4200L)
    #define pReset_USBIO_ENABLE               ((uint32)0x80000000u)
    #define pReset_USBIO_DISABLE              ((uint32)(~pReset_USBIO_ENABLE))
    #define pReset_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define pReset_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define pReset_USBIO_ENTER_SLEEP          ((uint32)((1u << pReset_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << pReset_USBIO_SUSPEND_DEL_SHIFT)))
    #define pReset_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << pReset_USBIO_SUSPEND_SHIFT)))
    #define pReset_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << pReset_USBIO_SUSPEND_DEL_SHIFT)))
    #define pReset_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(pReset__PC)
    /* Port Configuration */
    #define pReset_PC                 (* (reg32 *) pReset__PC)
#endif
/* Pin State */
#define pReset_PS                     (* (reg32 *) pReset__PS)
/* Data Register */
#define pReset_DR                     (* (reg32 *) pReset__DR)
/* Input Buffer Disable Override */
#define pReset_INP_DIS                (* (reg32 *) pReset__PC2)

/* Interrupt configuration Registers */
#define pReset_INTCFG                 (* (reg32 *) pReset__INTCFG)
#define pReset_INTSTAT                (* (reg32 *) pReset__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define pReset_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(pReset__SIO)
    #define pReset_SIO_REG            (* (reg32 *) pReset__SIO)
#endif /* (pReset__SIO_CFG) */

/* USBIO registers */
#if !defined(pReset__PC) && (CY_PSOC4_4200L)
    #define pReset_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define pReset_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define pReset_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define pReset_DRIVE_MODE_SHIFT       (0x00u)
#define pReset_DRIVE_MODE_MASK        (0x07u << pReset_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins pReset_H */


/* [] END OF FILE */
