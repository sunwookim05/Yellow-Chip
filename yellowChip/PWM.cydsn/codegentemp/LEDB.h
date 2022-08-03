/*******************************************************************************
* File Name: LEDB.h  
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

#if !defined(CY_PINS_LEDB_H) /* Pins LEDB_H */
#define CY_PINS_LEDB_H

#include "cytypes.h"
#include "cyfitter.h"
#include "LEDB_aliases.h"


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
} LEDB_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   LEDB_Read(void);
void    LEDB_Write(uint8 value);
uint8   LEDB_ReadDataReg(void);
#if defined(LEDB__PC) || (CY_PSOC4_4200L) 
    void    LEDB_SetDriveMode(uint8 mode);
#endif
void    LEDB_SetInterruptMode(uint16 position, uint16 mode);
uint8   LEDB_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void LEDB_Sleep(void); 
void LEDB_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(LEDB__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define LEDB_DRIVE_MODE_BITS        (3)
    #define LEDB_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - LEDB_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the LEDB_SetDriveMode() function.
         *  @{
         */
        #define LEDB_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define LEDB_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define LEDB_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define LEDB_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define LEDB_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define LEDB_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define LEDB_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define LEDB_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define LEDB_MASK               LEDB__MASK
#define LEDB_SHIFT              LEDB__SHIFT
#define LEDB_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in LEDB_SetInterruptMode() function.
     *  @{
     */
        #define LEDB_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define LEDB_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define LEDB_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define LEDB_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(LEDB__SIO)
    #define LEDB_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(LEDB__PC) && (CY_PSOC4_4200L)
    #define LEDB_USBIO_ENABLE               ((uint32)0x80000000u)
    #define LEDB_USBIO_DISABLE              ((uint32)(~LEDB_USBIO_ENABLE))
    #define LEDB_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define LEDB_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define LEDB_USBIO_ENTER_SLEEP          ((uint32)((1u << LEDB_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << LEDB_USBIO_SUSPEND_DEL_SHIFT)))
    #define LEDB_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << LEDB_USBIO_SUSPEND_SHIFT)))
    #define LEDB_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << LEDB_USBIO_SUSPEND_DEL_SHIFT)))
    #define LEDB_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(LEDB__PC)
    /* Port Configuration */
    #define LEDB_PC                 (* (reg32 *) LEDB__PC)
#endif
/* Pin State */
#define LEDB_PS                     (* (reg32 *) LEDB__PS)
/* Data Register */
#define LEDB_DR                     (* (reg32 *) LEDB__DR)
/* Input Buffer Disable Override */
#define LEDB_INP_DIS                (* (reg32 *) LEDB__PC2)

/* Interrupt configuration Registers */
#define LEDB_INTCFG                 (* (reg32 *) LEDB__INTCFG)
#define LEDB_INTSTAT                (* (reg32 *) LEDB__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define LEDB_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(LEDB__SIO)
    #define LEDB_SIO_REG            (* (reg32 *) LEDB__SIO)
#endif /* (LEDB__SIO_CFG) */

/* USBIO registers */
#if !defined(LEDB__PC) && (CY_PSOC4_4200L)
    #define LEDB_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define LEDB_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define LEDB_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define LEDB_DRIVE_MODE_SHIFT       (0x00u)
#define LEDB_DRIVE_MODE_MASK        (0x07u << LEDB_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins LEDB_H */


/* [] END OF FILE */
