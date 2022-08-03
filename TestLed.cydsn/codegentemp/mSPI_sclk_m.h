/*******************************************************************************
* File Name: mSPI_sclk_m.h  
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

#if !defined(CY_PINS_mSPI_sclk_m_H) /* Pins mSPI_sclk_m_H */
#define CY_PINS_mSPI_sclk_m_H

#include "cytypes.h"
#include "cyfitter.h"
#include "mSPI_sclk_m_aliases.h"


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
} mSPI_sclk_m_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   mSPI_sclk_m_Read(void);
void    mSPI_sclk_m_Write(uint8 value);
uint8   mSPI_sclk_m_ReadDataReg(void);
#if defined(mSPI_sclk_m__PC) || (CY_PSOC4_4200L) 
    void    mSPI_sclk_m_SetDriveMode(uint8 mode);
#endif
void    mSPI_sclk_m_SetInterruptMode(uint16 position, uint16 mode);
uint8   mSPI_sclk_m_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void mSPI_sclk_m_Sleep(void); 
void mSPI_sclk_m_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(mSPI_sclk_m__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define mSPI_sclk_m_DRIVE_MODE_BITS        (3)
    #define mSPI_sclk_m_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - mSPI_sclk_m_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the mSPI_sclk_m_SetDriveMode() function.
         *  @{
         */
        #define mSPI_sclk_m_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define mSPI_sclk_m_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define mSPI_sclk_m_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define mSPI_sclk_m_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define mSPI_sclk_m_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define mSPI_sclk_m_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define mSPI_sclk_m_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define mSPI_sclk_m_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define mSPI_sclk_m_MASK               mSPI_sclk_m__MASK
#define mSPI_sclk_m_SHIFT              mSPI_sclk_m__SHIFT
#define mSPI_sclk_m_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in mSPI_sclk_m_SetInterruptMode() function.
     *  @{
     */
        #define mSPI_sclk_m_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define mSPI_sclk_m_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define mSPI_sclk_m_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define mSPI_sclk_m_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(mSPI_sclk_m__SIO)
    #define mSPI_sclk_m_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(mSPI_sclk_m__PC) && (CY_PSOC4_4200L)
    #define mSPI_sclk_m_USBIO_ENABLE               ((uint32)0x80000000u)
    #define mSPI_sclk_m_USBIO_DISABLE              ((uint32)(~mSPI_sclk_m_USBIO_ENABLE))
    #define mSPI_sclk_m_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define mSPI_sclk_m_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define mSPI_sclk_m_USBIO_ENTER_SLEEP          ((uint32)((1u << mSPI_sclk_m_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << mSPI_sclk_m_USBIO_SUSPEND_DEL_SHIFT)))
    #define mSPI_sclk_m_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << mSPI_sclk_m_USBIO_SUSPEND_SHIFT)))
    #define mSPI_sclk_m_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << mSPI_sclk_m_USBIO_SUSPEND_DEL_SHIFT)))
    #define mSPI_sclk_m_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(mSPI_sclk_m__PC)
    /* Port Configuration */
    #define mSPI_sclk_m_PC                 (* (reg32 *) mSPI_sclk_m__PC)
#endif
/* Pin State */
#define mSPI_sclk_m_PS                     (* (reg32 *) mSPI_sclk_m__PS)
/* Data Register */
#define mSPI_sclk_m_DR                     (* (reg32 *) mSPI_sclk_m__DR)
/* Input Buffer Disable Override */
#define mSPI_sclk_m_INP_DIS                (* (reg32 *) mSPI_sclk_m__PC2)

/* Interrupt configuration Registers */
#define mSPI_sclk_m_INTCFG                 (* (reg32 *) mSPI_sclk_m__INTCFG)
#define mSPI_sclk_m_INTSTAT                (* (reg32 *) mSPI_sclk_m__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define mSPI_sclk_m_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(mSPI_sclk_m__SIO)
    #define mSPI_sclk_m_SIO_REG            (* (reg32 *) mSPI_sclk_m__SIO)
#endif /* (mSPI_sclk_m__SIO_CFG) */

/* USBIO registers */
#if !defined(mSPI_sclk_m__PC) && (CY_PSOC4_4200L)
    #define mSPI_sclk_m_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define mSPI_sclk_m_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define mSPI_sclk_m_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define mSPI_sclk_m_DRIVE_MODE_SHIFT       (0x00u)
#define mSPI_sclk_m_DRIVE_MODE_MASK        (0x07u << mSPI_sclk_m_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins mSPI_sclk_m_H */


/* [] END OF FILE */
