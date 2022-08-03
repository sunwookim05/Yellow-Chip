/***************************************************************************//**
* \file mSPI.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the SCB Component.
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

#if !defined(CY_SCB_mSPI_H)
#define CY_SCB_mSPI_H

#include <cydevice_trm.h>
#include <cyfitter.h>
#include <cytypes.h>
#include <CyLib.h>

/* SCB IP block v0 is available in PSoC 4100/PSoC 4200 */
#define mSPI_CY_SCBIP_V0    (CYIPBLOCK_m0s8scb_VERSION == 0u)
/* SCB IP block v1 is available in PSoC 4000 */
#define mSPI_CY_SCBIP_V1    (CYIPBLOCK_m0s8scb_VERSION == 1u)
/* SCB IP block v2 is available in all other devices */
#define mSPI_CY_SCBIP_V2    (CYIPBLOCK_m0s8scb_VERSION >= 2u)

/** Component version major.minor */
#define mSPI_COMP_VERSION_MAJOR    (4)
#define mSPI_COMP_VERSION_MINOR    (0)
    
#define mSPI_SCB_MODE           (2u)

/* SCB modes enum */
#define mSPI_SCB_MODE_I2C       (0x01u)
#define mSPI_SCB_MODE_SPI       (0x02u)
#define mSPI_SCB_MODE_UART      (0x04u)
#define mSPI_SCB_MODE_EZI2C     (0x08u)
#define mSPI_SCB_MODE_UNCONFIG  (0xFFu)

/* Condition compilation depends on operation mode: Unconfigured implies apply to all modes */
#define mSPI_SCB_MODE_I2C_CONST_CFG       (mSPI_SCB_MODE_I2C       == mSPI_SCB_MODE)
#define mSPI_SCB_MODE_SPI_CONST_CFG       (mSPI_SCB_MODE_SPI       == mSPI_SCB_MODE)
#define mSPI_SCB_MODE_UART_CONST_CFG      (mSPI_SCB_MODE_UART      == mSPI_SCB_MODE)
#define mSPI_SCB_MODE_EZI2C_CONST_CFG     (mSPI_SCB_MODE_EZI2C     == mSPI_SCB_MODE)
#define mSPI_SCB_MODE_UNCONFIG_CONST_CFG  (mSPI_SCB_MODE_UNCONFIG  == mSPI_SCB_MODE)

/* Condition compilation for includes */
#define mSPI_SCB_MODE_I2C_INC      (0u !=(mSPI_SCB_MODE_I2C   & mSPI_SCB_MODE))
#define mSPI_SCB_MODE_EZI2C_INC    (0u !=(mSPI_SCB_MODE_EZI2C & mSPI_SCB_MODE))
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_SCB_MODE_SPI_INC  (0u !=(mSPI_SCB_MODE_SPI   & mSPI_SCB_MODE))
    #define mSPI_SCB_MODE_UART_INC (0u !=(mSPI_SCB_MODE_UART  & mSPI_SCB_MODE))
#else
    #define mSPI_SCB_MODE_SPI_INC  (0u)
    #define mSPI_SCB_MODE_UART_INC (0u)
#endif /* (!mSPI_CY_SCBIP_V1) */

/* Interrupts remove options */
#define mSPI_REMOVE_SCB_IRQ             (1u)
#define mSPI_SCB_IRQ_INTERNAL           (0u == mSPI_REMOVE_SCB_IRQ)

#define mSPI_REMOVE_UART_RX_WAKEUP_IRQ  (1u)
#define mSPI_UART_RX_WAKEUP_IRQ         (0u == mSPI_REMOVE_UART_RX_WAKEUP_IRQ)

/* SCB interrupt enum */
#define mSPI_SCB_INTR_MODE_NONE     (0u)
#define mSPI_SCB_INTR_MODE_INTERNAL (1u)
#define mSPI_SCB_INTR_MODE_EXTERNAL (2u)

/* Internal clock remove option */
#define mSPI_REMOVE_SCB_CLK     (0u)
#define mSPI_SCB_CLK_INTERNAL   (0u == mSPI_REMOVE_SCB_CLK)


/***************************************
*       Includes
****************************************/

#include "mSPI_PINS.h"

#if (mSPI_SCB_CLK_INTERNAL)
    #include "mSPI_SCBCLK.h"
#endif /* (mSPI_SCB_CLK_INTERNAL) */


/***************************************
*       Type Definitions
***************************************/

typedef struct
{
    uint8 enableState;
} mSPI_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/**
* \addtogroup group_general
* @{
*/

/* Start and Stop APIs */
void mSPI_Init(void);
void mSPI_Enable(void);
void mSPI_Start(void);
void mSPI_Stop(void);

/** @} general */

/**
* \addtogroup group_power
* @{
*/
/* Sleep and Wakeup APis */
void mSPI_Sleep(void);
void mSPI_Wakeup(void);
/** @} power */ 

/**
* \addtogroup group_interrupt
* @{
*/
#if (mSPI_SCB_IRQ_INTERNAL)
    /* Custom interrupt handler */
    void mSPI_SetCustomInterruptHandler(void (*func)(void));
#endif /* (mSPI_SCB_IRQ_INTERNAL) */
/** @} interrupt */

/* Interface to internal interrupt component */
#if (mSPI_SCB_IRQ_INTERNAL)
    /**
    * \addtogroup group_interrupt
    * @{
    */    
    /*******************************************************************************
    * Function Name: mSPI_EnableInt
    ****************************************************************************//**
    *
    *  When using an Internal interrupt, this enables the interrupt in the NVIC. 
    *  When using an external interrupt the API for the interrupt component must 
    *  be used to enable the interrupt.
    *
    *******************************************************************************/
    #define mSPI_EnableInt()    CyIntEnable(mSPI_ISR_NUMBER)
    
    
    /*******************************************************************************
    * Function Name: mSPI_DisableInt
    ****************************************************************************//**
    *
    *  When using an Internal interrupt, this disables the interrupt in the NVIC. 
    *  When using an external interrupt the API for the interrupt component must 
    *  be used to disable the interrupt.
    *
    *******************************************************************************/    
    #define mSPI_DisableInt()   CyIntDisable(mSPI_ISR_NUMBER)
    /** @} interrupt */

    /*******************************************************************************
    * Function Name: mSPI_ClearPendingInt
    ****************************************************************************//**
    *
    *  This function clears the interrupt pending status in the NVIC. 
    *
    *******************************************************************************/
    #define mSPI_ClearPendingInt()  CyIntClearPending(mSPI_ISR_NUMBER)
#endif /* (mSPI_SCB_IRQ_INTERNAL) */

#if (mSPI_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: mSPI_RxWakeEnableInt
    ****************************************************************************//**
    *
    *  This function enables the interrupt (RX_WAKE) pending status in the NVIC. 
    *
    *******************************************************************************/    
    #define mSPI_RxWakeEnableInt()  CyIntEnable(mSPI_RX_WAKE_ISR_NUMBER)
    

    /*******************************************************************************
    * Function Name: mSPI_RxWakeDisableInt
    ****************************************************************************//**
    *
    *  This function disables the interrupt (RX_WAKE) pending status in the NVIC.  
    *
    *******************************************************************************/
    #define mSPI_RxWakeDisableInt() CyIntDisable(mSPI_RX_WAKE_ISR_NUMBER)
    
    
    /*******************************************************************************
    * Function Name: mSPI_RxWakeClearPendingInt
    ****************************************************************************//**
    *
    *  This function clears the interrupt (RX_WAKE) pending status in the NVIC. 
    *
    *******************************************************************************/    
    #define mSPI_RxWakeClearPendingInt()  CyIntClearPending(mSPI_RX_WAKE_ISR_NUMBER)
#endif /* (mSPI_UART_RX_WAKEUP_IRQ) */

/**
* \addtogroup group_interrupt
* @{
*/
/* Get interrupt cause */
/*******************************************************************************
* Function Name: mSPI_GetInterruptCause
****************************************************************************//**
*
*  Returns a mask of bits showing the source of the current triggered interrupt. 
*  This is useful for modes of operation where an interrupt can be generated by 
*  conditions in multiple interrupt source registers.
*
*  \return
*   Mask with the OR of the following conditions that have been triggered.
*    - mSPI_INTR_CAUSE_MASTER - Interrupt from Master
*    - mSPI_INTR_CAUSE_SLAVE - Interrupt from Slave
*    - mSPI_INTR_CAUSE_TX - Interrupt from TX
*    - mSPI_INTR_CAUSE_RX - Interrupt from RX
*
*******************************************************************************/
#define mSPI_GetInterruptCause()    (mSPI_INTR_CAUSE_REG)


/* APIs to service INTR_RX register */
/*******************************************************************************
* Function Name: mSPI_GetRxInterruptSource
****************************************************************************//**
*
*  Returns RX interrupt request register. This register contains current status 
*  of RX interrupt sources.
*
*  \return
*   Current status of RX interrupt sources.
*   Each constant is a bit field value. The value returned may have multiple 
*   bits set to indicate the current status.
*   - mSPI_INTR_RX_FIFO_LEVEL - The number of data elements in the 
      RX FIFO is greater than the value of RX FIFO level.
*   - mSPI_INTR_RX_NOT_EMPTY - Receiver FIFO is not empty.
*   - mSPI_INTR_RX_FULL - Receiver FIFO is full.
*   - mSPI_INTR_RX_OVERFLOW - Attempt to write to a full 
*     receiver FIFO.
*   - mSPI_INTR_RX_UNDERFLOW - Attempt to read from an empty 
*     receiver FIFO.
*   - mSPI_INTR_RX_FRAME_ERROR - UART framing error detected.
*   - mSPI_INTR_RX_PARITY_ERROR - UART parity error detected.
*
*******************************************************************************/
#define mSPI_GetRxInterruptSource() (mSPI_INTR_RX_REG)


/*******************************************************************************
* Function Name: mSPI_SetRxInterruptMode
****************************************************************************//**
*
*  Writes RX interrupt mask register. This register configures which bits from 
*  RX interrupt request register will trigger an interrupt event.
*
*  \param interruptMask: RX interrupt sources to be enabled (refer to 
*   mSPI_GetRxInterruptSource() function for bit fields values).
*
*******************************************************************************/
#define mSPI_SetRxInterruptMode(interruptMask)     mSPI_WRITE_INTR_RX_MASK(interruptMask)


/*******************************************************************************
* Function Name: mSPI_GetRxInterruptMode
****************************************************************************//**
*
*  Returns RX interrupt mask register This register specifies which bits from 
*  RX interrupt request register will trigger an interrupt event.
*
*  \return 
*   RX interrupt sources to be enabled (refer to 
*   mSPI_GetRxInterruptSource() function for bit fields values).
*
*******************************************************************************/
#define mSPI_GetRxInterruptMode()   (mSPI_INTR_RX_MASK_REG)


/*******************************************************************************
* Function Name: mSPI_GetRxInterruptSourceMasked
****************************************************************************//**
*
*  Returns RX interrupt masked request register. This register contains logical
*  AND of corresponding bits from RX interrupt request and mask registers.
*  This function is intended to be used in the interrupt service routine to 
*  identify which of enabled RX interrupt sources cause interrupt event.
*
*  \return 
*   Current status of enabled RX interrupt sources (refer to 
*   mSPI_GetRxInterruptSource() function for bit fields values).
*
*******************************************************************************/
#define mSPI_GetRxInterruptSourceMasked()   (mSPI_INTR_RX_MASKED_REG)


/*******************************************************************************
* Function Name: mSPI_ClearRxInterruptSource
****************************************************************************//**
*
*  Clears RX interrupt sources in the interrupt request register.
*
*  \param interruptMask: RX interrupt sources to be cleared (refer to 
*   mSPI_GetRxInterruptSource() function for bit fields values).
*
*  \sideeffects 
*   The side effects are listed in the table below for each 
*   affected interrupt source. Refer to section RX FIFO interrupt sources for 
*   detailed description.
*   - mSPI_INTR_RX_FIFO_LEVEL Interrupt source is not cleared when 
*     the receiver FIFO has more entries than level.
*   - mSPI_INTR_RX_NOT_EMPTY Interrupt source is not cleared when
*     receiver FIFO is not empty.
*   - mSPI_INTR_RX_FULL Interrupt source is not cleared when 
*      receiver FIFO is full.
*
*******************************************************************************/
#define mSPI_ClearRxInterruptSource(interruptMask)  mSPI_CLEAR_INTR_RX(interruptMask)


/*******************************************************************************
* Function Name: mSPI_SetRxInterrupt
****************************************************************************//**
*
*  Sets RX interrupt sources in the interrupt request register.
*
*  \param interruptMask: RX interrupt sources to set in the RX interrupt request 
*   register (refer to mSPI_GetRxInterruptSource() function for bit 
*   fields values).
*
*******************************************************************************/
#define mSPI_SetRxInterrupt(interruptMask)  mSPI_SET_INTR_RX(interruptMask)

void mSPI_SetRxFifoLevel(uint32 level);


/* APIs to service INTR_TX register */
/*******************************************************************************
* Function Name: mSPI_GetTxInterruptSource
****************************************************************************//**
*
*  Returns TX interrupt request register. This register contains current status 
*  of TX interrupt sources.
* 
*  \return 
*   Current status of TX interrupt sources.
*   Each constant is a bit field value. The value returned may have multiple 
*   bits set to indicate the current status.
*   - mSPI_INTR_TX_FIFO_LEVEL - The number of data elements in the 
*     TX FIFO is less than the value of TX FIFO level.
*   - mSPI_INTR_TX_NOT_FULL - Transmitter FIFO is not full.
*   - mSPI_INTR_TX_EMPTY - Transmitter FIFO is empty.
*   - mSPI_INTR_TX_OVERFLOW - Attempt to write to a full 
*     transmitter FIFO.
*   - mSPI_INTR_TX_UNDERFLOW - Attempt to read from an empty 
*     transmitter FIFO.
*   - mSPI_INTR_TX_UART_NACK - UART received a NACK in SmartCard 
*   mode.
*   - mSPI_INTR_TX_UART_DONE - UART transfer is complete. 
*     All data elements from the TX FIFO are sent.
*   - mSPI_INTR_TX_UART_ARB_LOST - Value on the TX line of the UART
*     does not match the value on the RX line.
*
*******************************************************************************/
#define mSPI_GetTxInterruptSource() (mSPI_INTR_TX_REG)


/*******************************************************************************
* Function Name: mSPI_SetTxInterruptMode
****************************************************************************//**
*
*  Writes TX interrupt mask register. This register configures which bits from 
*  TX interrupt request register will trigger an interrupt event.
*
*  \param interruptMask: TX interrupt sources to be enabled (refer to 
*   mSPI_GetTxInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_SetTxInterruptMode(interruptMask)  mSPI_WRITE_INTR_TX_MASK(interruptMask)


/*******************************************************************************
* Function Name: mSPI_GetTxInterruptMode
****************************************************************************//**
*
*  Returns TX interrupt mask register This register specifies which bits from 
*  TX interrupt request register will trigger an interrupt event.
*
*  \return 
*   Enabled TX interrupt sources (refer to 
*   mSPI_GetTxInterruptSource() function for bit field values).
*   
*******************************************************************************/
#define mSPI_GetTxInterruptMode()   (mSPI_INTR_TX_MASK_REG)


/*******************************************************************************
* Function Name: mSPI_GetTxInterruptSourceMasked
****************************************************************************//**
*
*  Returns TX interrupt masked request register. This register contains logical
*  AND of corresponding bits from TX interrupt request and mask registers.
*  This function is intended to be used in the interrupt service routine to identify 
*  which of enabled TX interrupt sources cause interrupt event.
*
*  \return 
*   Current status of enabled TX interrupt sources (refer to 
*   mSPI_GetTxInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_GetTxInterruptSourceMasked()   (mSPI_INTR_TX_MASKED_REG)


/*******************************************************************************
* Function Name: mSPI_ClearTxInterruptSource
****************************************************************************//**
*
*  Clears TX interrupt sources in the interrupt request register.
*
*  \param interruptMask: TX interrupt sources to be cleared (refer to 
*   mSPI_GetTxInterruptSource() function for bit field values).
*
*  \sideeffects 
*   The side effects are listed in the table below for each affected interrupt 
*   source. Refer to section TX FIFO interrupt sources for detailed description.
*   - mSPI_INTR_TX_FIFO_LEVEL - Interrupt source is not cleared when 
*     transmitter FIFO has less entries than level.
*   - mSPI_INTR_TX_NOT_FULL - Interrupt source is not cleared when
*     transmitter FIFO has empty entries.
*   - mSPI_INTR_TX_EMPTY - Interrupt source is not cleared when 
*     transmitter FIFO is empty.
*   - mSPI_INTR_TX_UNDERFLOW - Interrupt source is not cleared when 
*     transmitter FIFO is empty and I2C mode with clock stretching is selected. 
*     Put data into the transmitter FIFO before clearing it. This behavior only 
*     applicable for PSoC 4100/PSoC 4200 devices.
*
*******************************************************************************/
#define mSPI_ClearTxInterruptSource(interruptMask)  mSPI_CLEAR_INTR_TX(interruptMask)


/*******************************************************************************
* Function Name: mSPI_SetTxInterrupt
****************************************************************************//**
*
*  Sets RX interrupt sources in the interrupt request register.
*
*  \param interruptMask: RX interrupt sources to set in the RX interrupt request 
*   register (refer to mSPI_GetRxInterruptSource() function for bit 
*   fields values).
*
*******************************************************************************/
#define mSPI_SetTxInterrupt(interruptMask)  mSPI_SET_INTR_TX(interruptMask)

void mSPI_SetTxFifoLevel(uint32 level);


/* APIs to service INTR_MASTER register */
/*******************************************************************************
* Function Name: mSPI_GetMasterInterruptSource
****************************************************************************//**
*
*  Returns Master interrupt request register. This register contains current 
*  status of Master interrupt sources.
*
*  \return 
*   Current status of Master interrupt sources. 
*   Each constant is a bit field value. The value returned may have multiple 
*   bits set to indicate the current status.
*   - mSPI_INTR_MASTER_SPI_DONE - SPI master transfer is complete.
*     Refer to Interrupt sources section for detailed description.
*   - mSPI_INTR_MASTER_I2C_ARB_LOST - I2C master lost arbitration.
*   - mSPI_INTR_MASTER_I2C_NACK - I2C master received negative 
*    acknowledgement (NAK).
*   - mSPI_INTR_MASTER_I2C_ACK - I2C master received acknowledgement.
*   - mSPI_INTR_MASTER_I2C_STOP - I2C master generated STOP.
*   - mSPI_INTR_MASTER_I2C_BUS_ERROR - I2C master bus error 
*     (detection of unexpected START or STOP condition).
*
*******************************************************************************/
#define mSPI_GetMasterInterruptSource() (mSPI_INTR_MASTER_REG)

/*******************************************************************************
* Function Name: mSPI_SetMasterInterruptMode
****************************************************************************//**
*
*  Writes Master interrupt mask register. This register configures which bits 
*  from Master interrupt request register will trigger an interrupt event.
*
*  \param interruptMask: Master interrupt sources to be enabled (refer to 
*   mSPI_GetMasterInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_SetMasterInterruptMode(interruptMask)  mSPI_WRITE_INTR_MASTER_MASK(interruptMask)

/*******************************************************************************
* Function Name: mSPI_GetMasterInterruptMode
****************************************************************************//**
*
*  Returns Master interrupt mask register This register specifies which bits 
*  from Master interrupt request register will trigger an interrupt event.
*
*  \return 
*   Enabled Master interrupt sources (refer to 
*   mSPI_GetMasterInterruptSource() function for return values).
*
*******************************************************************************/
#define mSPI_GetMasterInterruptMode()   (mSPI_INTR_MASTER_MASK_REG)

/*******************************************************************************
* Function Name: mSPI_GetMasterInterruptSourceMasked
****************************************************************************//**
*
*  Returns Master interrupt masked request register. This register contains 
*  logical AND of corresponding bits from Master interrupt request and mask 
*  registers.
*  This function is intended to be used in the interrupt service routine to 
*  identify which of enabled Master interrupt sources cause interrupt event.
*
*  \return 
*   Current status of enabled Master interrupt sources (refer to 
*   mSPI_GetMasterInterruptSource() function for return values).
*
*******************************************************************************/
#define mSPI_GetMasterInterruptSourceMasked()   (mSPI_INTR_MASTER_MASKED_REG)

/*******************************************************************************
* Function Name: mSPI_ClearMasterInterruptSource
****************************************************************************//**
*
*  Clears Master interrupt sources in the interrupt request register.
*
*  \param interruptMask: Master interrupt sources to be cleared (refer to 
*   mSPI_GetMasterInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_ClearMasterInterruptSource(interruptMask)  mSPI_CLEAR_INTR_MASTER(interruptMask)

/*******************************************************************************
* Function Name: mSPI_SetMasterInterrupt
****************************************************************************//**
*
*  Sets Master interrupt sources in the interrupt request register.
*
*  \param interruptMask: Master interrupt sources to set in the Master interrupt
*   request register (refer to mSPI_GetMasterInterruptSource() 
*   function for bit field values).
*
*******************************************************************************/
#define mSPI_SetMasterInterrupt(interruptMask)  mSPI_SET_INTR_MASTER(interruptMask)


/* APIs to service INTR_SLAVE register */
/*******************************************************************************
* Function Name: mSPI_GetSlaveInterruptSource
****************************************************************************//**
*
*  Returns Slave interrupt request register. This register contains current 
*  status of Slave interrupt sources.
*
*  \return 
*   Current status of Slave interrupt sources.
*   Each constant is a bit field value. The value returned may have multiple 
*   bits set to indicate the current status.
*   - mSPI_INTR_SLAVE_I2C_ARB_LOST - I2C slave lost arbitration: 
*     the value driven on the SDA line is not the same as the value observed 
*     on the SDA line.
*   - mSPI_INTR_SLAVE_I2C_NACK - I2C slave received negative 
*     acknowledgement (NAK).
*   - mSPI_INTR_SLAVE_I2C_ACK - I2C slave received 
*     acknowledgement (ACK).
*   - mSPI_INTR_SLAVE_I2C_WRITE_STOP - Stop or Repeated Start 
*     event for write transfer intended for this slave (address matching 
*     is performed).
*   - mSPI_INTR_SLAVE_I2C_STOP - Stop or Repeated Start event 
*     for (read or write) transfer intended for this slave (address matching 
*     is performed).
*   - mSPI_INTR_SLAVE_I2C_START - I2C slave received Start 
*     condition.
*   - mSPI_INTR_SLAVE_I2C_ADDR_MATCH - I2C slave received matching 
*     address.
*   - mSPI_INTR_SLAVE_I2C_GENERAL - I2C Slave received general 
*     call address.
*   - mSPI_INTR_SLAVE_I2C_BUS_ERROR - I2C slave bus error (detection 
*      of unexpected Start or Stop condition).
*   - mSPI_INTR_SLAVE_SPI_BUS_ERROR - SPI slave select line is 
*      deselected at an expected time while the SPI transfer.
*
*******************************************************************************/
#define mSPI_GetSlaveInterruptSource()  (mSPI_INTR_SLAVE_REG)

/*******************************************************************************
* Function Name: mSPI_SetSlaveInterruptMode
****************************************************************************//**
*
*  Writes Slave interrupt mask register. 
*  This register configures which bits from Slave interrupt request register 
*  will trigger an interrupt event.
*
*  \param interruptMask: Slave interrupt sources to be enabled (refer to 
*   mSPI_GetSlaveInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_SetSlaveInterruptMode(interruptMask)   mSPI_WRITE_INTR_SLAVE_MASK(interruptMask)

/*******************************************************************************
* Function Name: mSPI_GetSlaveInterruptMode
****************************************************************************//**
*
*  Returns Slave interrupt mask register.
*  This register specifies which bits from Slave interrupt request register 
*  will trigger an interrupt event.
*
*  \return 
*   Enabled Slave interrupt sources(refer to 
*   mSPI_GetSlaveInterruptSource() function for bit field values).
*
*******************************************************************************/
#define mSPI_GetSlaveInterruptMode()    (mSPI_INTR_SLAVE_MASK_REG)

/*******************************************************************************
* Function Name: mSPI_GetSlaveInterruptSourceMasked
****************************************************************************//**
*
*  Returns Slave interrupt masked request register. This register contains 
*  logical AND of corresponding bits from Slave interrupt request and mask 
*  registers.
*  This function is intended to be used in the interrupt service routine to 
*  identify which of enabled Slave interrupt sources cause interrupt event.
*
*  \return 
*   Current status of enabled Slave interrupt sources (refer to 
*   mSPI_GetSlaveInterruptSource() function for return values).
*
*******************************************************************************/
#define mSPI_GetSlaveInterruptSourceMasked()    (mSPI_INTR_SLAVE_MASKED_REG)

/*******************************************************************************
* Function Name: mSPI_ClearSlaveInterruptSource
****************************************************************************//**
*
*  Clears Slave interrupt sources in the interrupt request register.
*
*  \param interruptMask: Slave interrupt sources to be cleared (refer to 
*   mSPI_GetSlaveInterruptSource() function for return values).
*
*******************************************************************************/
#define mSPI_ClearSlaveInterruptSource(interruptMask)   mSPI_CLEAR_INTR_SLAVE(interruptMask)

/*******************************************************************************
* Function Name: mSPI_SetSlaveInterrupt
****************************************************************************//**
*
*  Sets Slave interrupt sources in the interrupt request register.
*
*  \param interruptMask: Slave interrupt sources to set in the Slave interrupt 
*   request register (refer to mSPI_GetSlaveInterruptSource() 
*   function for return values).
*
*******************************************************************************/
#define mSPI_SetSlaveInterrupt(interruptMask)   mSPI_SET_INTR_SLAVE(interruptMask)

/** @} interrupt */ 


/***************************************
*     Vars with External Linkage
***************************************/

/**
* \addtogroup group_globals
* @{
*/

/** mSPI_initVar indicates whether the mSPI 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the mSPI_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  mSPI_Init() function can be called before the 
*  mSPI_Start() or mSPI_Enable() function.
*/
extern uint8 mSPI_initVar;
/** @} globals */

/***************************************
*              Registers
***************************************/

#define mSPI_CTRL_REG               (*(reg32 *) mSPI_SCB__CTRL)
#define mSPI_CTRL_PTR               ( (reg32 *) mSPI_SCB__CTRL)

#define mSPI_STATUS_REG             (*(reg32 *) mSPI_SCB__STATUS)
#define mSPI_STATUS_PTR             ( (reg32 *) mSPI_SCB__STATUS)

#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_SPI_CTRL_REG           (*(reg32 *) mSPI_SCB__SPI_CTRL)
    #define mSPI_SPI_CTRL_PTR           ( (reg32 *) mSPI_SCB__SPI_CTRL)

    #define mSPI_SPI_STATUS_REG         (*(reg32 *) mSPI_SCB__SPI_STATUS)
    #define mSPI_SPI_STATUS_PTR         ( (reg32 *) mSPI_SCB__SPI_STATUS)

    #define mSPI_UART_CTRL_REG          (*(reg32 *) mSPI_SCB__UART_CTRL)
    #define mSPI_UART_CTRL_PTR          ( (reg32 *) mSPI_SCB__UART_CTRL)

    #define mSPI_UART_TX_CTRL_REG       (*(reg32 *) mSPI_SCB__UART_TX_CTRL)
    #define mSPI_UART_TX_CTRL_PTR       ( (reg32 *) mSPI_SCB__UART_TX_CTRL)

    #define mSPI_UART_RX_CTRL_REG       (*(reg32 *) mSPI_SCB__UART_RX_CTRL)
    #define mSPI_UART_RX_CTRL_PTR       ( (reg32 *) mSPI_SCB__UART_RX_CTRL)

    #define mSPI_UART_RX_STATUS_REG     (*(reg32 *) mSPI_SCB__UART_RX_STATUS)
    #define mSPI_UART_RX_STATUS_PTR     ( (reg32 *) mSPI_SCB__UART_RX_STATUS)
#endif /* (!mSPI_CY_SCBIP_V1) */

#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_UART_FLOW_CTRL_REG     (*(reg32 *) mSPI_SCB__UART_FLOW_CTRL)
    #define mSPI_UART_FLOW_CTRL_PTR     ( (reg32 *) mSPI_SCB__UART_FLOW_CTRL)
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

#define mSPI_I2C_CTRL_REG           (*(reg32 *) mSPI_SCB__I2C_CTRL)
#define mSPI_I2C_CTRL_PTR           ( (reg32 *) mSPI_SCB__I2C_CTRL)

#define mSPI_I2C_STATUS_REG         (*(reg32 *) mSPI_SCB__I2C_STATUS)
#define mSPI_I2C_STATUS_PTR         ( (reg32 *) mSPI_SCB__I2C_STATUS)

#define mSPI_I2C_MASTER_CMD_REG     (*(reg32 *) mSPI_SCB__I2C_M_CMD)
#define mSPI_I2C_MASTER_CMD_PTR     ( (reg32 *) mSPI_SCB__I2C_M_CMD)

#define mSPI_I2C_SLAVE_CMD_REG      (*(reg32 *) mSPI_SCB__I2C_S_CMD)
#define mSPI_I2C_SLAVE_CMD_PTR      ( (reg32 *) mSPI_SCB__I2C_S_CMD)

#define mSPI_I2C_CFG_REG            (*(reg32 *) mSPI_SCB__I2C_CFG)
#define mSPI_I2C_CFG_PTR            ( (reg32 *) mSPI_SCB__I2C_CFG)

#define mSPI_TX_CTRL_REG            (*(reg32 *) mSPI_SCB__TX_CTRL)
#define mSPI_TX_CTRL_PTR            ( (reg32 *) mSPI_SCB__TX_CTRL)

#define mSPI_TX_FIFO_CTRL_REG       (*(reg32 *) mSPI_SCB__TX_FIFO_CTRL)
#define mSPI_TX_FIFO_CTRL_PTR       ( (reg32 *) mSPI_SCB__TX_FIFO_CTRL)

#define mSPI_TX_FIFO_STATUS_REG     (*(reg32 *) mSPI_SCB__TX_FIFO_STATUS)
#define mSPI_TX_FIFO_STATUS_PTR     ( (reg32 *) mSPI_SCB__TX_FIFO_STATUS)

#define mSPI_TX_FIFO_WR_REG         (*(reg32 *) mSPI_SCB__TX_FIFO_WR)
#define mSPI_TX_FIFO_WR_PTR         ( (reg32 *) mSPI_SCB__TX_FIFO_WR)

#define mSPI_RX_CTRL_REG            (*(reg32 *) mSPI_SCB__RX_CTRL)
#define mSPI_RX_CTRL_PTR            ( (reg32 *) mSPI_SCB__RX_CTRL)

#define mSPI_RX_FIFO_CTRL_REG       (*(reg32 *) mSPI_SCB__RX_FIFO_CTRL)
#define mSPI_RX_FIFO_CTRL_PTR       ( (reg32 *) mSPI_SCB__RX_FIFO_CTRL)

#define mSPI_RX_FIFO_STATUS_REG     (*(reg32 *) mSPI_SCB__RX_FIFO_STATUS)
#define mSPI_RX_FIFO_STATUS_PTR     ( (reg32 *) mSPI_SCB__RX_FIFO_STATUS)

#define mSPI_RX_MATCH_REG           (*(reg32 *) mSPI_SCB__RX_MATCH)
#define mSPI_RX_MATCH_PTR           ( (reg32 *) mSPI_SCB__RX_MATCH)

#define mSPI_RX_FIFO_RD_REG         (*(reg32 *) mSPI_SCB__RX_FIFO_RD)
#define mSPI_RX_FIFO_RD_PTR         ( (reg32 *) mSPI_SCB__RX_FIFO_RD)

#define mSPI_RX_FIFO_RD_SILENT_REG  (*(reg32 *) mSPI_SCB__RX_FIFO_RD_SILENT)
#define mSPI_RX_FIFO_RD_SILENT_PTR  ( (reg32 *) mSPI_SCB__RX_FIFO_RD_SILENT)

#ifdef mSPI_SCB__EZ_DATA0
    #define mSPI_EZBUF_DATA0_REG    (*(reg32 *) mSPI_SCB__EZ_DATA0)
    #define mSPI_EZBUF_DATA0_PTR    ( (reg32 *) mSPI_SCB__EZ_DATA0)
#else
    #define mSPI_EZBUF_DATA0_REG    (*(reg32 *) mSPI_SCB__EZ_DATA00)
    #define mSPI_EZBUF_DATA0_PTR    ( (reg32 *) mSPI_SCB__EZ_DATA00)
#endif /* mSPI_SCB__EZ_DATA00 */

#define mSPI_INTR_CAUSE_REG         (*(reg32 *) mSPI_SCB__INTR_CAUSE)
#define mSPI_INTR_CAUSE_PTR         ( (reg32 *) mSPI_SCB__INTR_CAUSE)

#define mSPI_INTR_I2C_EC_REG        (*(reg32 *) mSPI_SCB__INTR_I2C_EC)
#define mSPI_INTR_I2C_EC_PTR        ( (reg32 *) mSPI_SCB__INTR_I2C_EC)

#define mSPI_INTR_I2C_EC_MASK_REG   (*(reg32 *) mSPI_SCB__INTR_I2C_EC_MASK)
#define mSPI_INTR_I2C_EC_MASK_PTR   ( (reg32 *) mSPI_SCB__INTR_I2C_EC_MASK)

#define mSPI_INTR_I2C_EC_MASKED_REG (*(reg32 *) mSPI_SCB__INTR_I2C_EC_MASKED)
#define mSPI_INTR_I2C_EC_MASKED_PTR ( (reg32 *) mSPI_SCB__INTR_I2C_EC_MASKED)

#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_INTR_SPI_EC_REG        (*(reg32 *) mSPI_SCB__INTR_SPI_EC)
    #define mSPI_INTR_SPI_EC_PTR        ( (reg32 *) mSPI_SCB__INTR_SPI_EC)

    #define mSPI_INTR_SPI_EC_MASK_REG   (*(reg32 *) mSPI_SCB__INTR_SPI_EC_MASK)
    #define mSPI_INTR_SPI_EC_MASK_PTR   ( (reg32 *) mSPI_SCB__INTR_SPI_EC_MASK)

    #define mSPI_INTR_SPI_EC_MASKED_REG (*(reg32 *) mSPI_SCB__INTR_SPI_EC_MASKED)
    #define mSPI_INTR_SPI_EC_MASKED_PTR ( (reg32 *) mSPI_SCB__INTR_SPI_EC_MASKED)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_INTR_MASTER_REG        (*(reg32 *) mSPI_SCB__INTR_M)
#define mSPI_INTR_MASTER_PTR        ( (reg32 *) mSPI_SCB__INTR_M)

#define mSPI_INTR_MASTER_SET_REG    (*(reg32 *) mSPI_SCB__INTR_M_SET)
#define mSPI_INTR_MASTER_SET_PTR    ( (reg32 *) mSPI_SCB__INTR_M_SET)

#define mSPI_INTR_MASTER_MASK_REG   (*(reg32 *) mSPI_SCB__INTR_M_MASK)
#define mSPI_INTR_MASTER_MASK_PTR   ( (reg32 *) mSPI_SCB__INTR_M_MASK)

#define mSPI_INTR_MASTER_MASKED_REG (*(reg32 *) mSPI_SCB__INTR_M_MASKED)
#define mSPI_INTR_MASTER_MASKED_PTR ( (reg32 *) mSPI_SCB__INTR_M_MASKED)

#define mSPI_INTR_SLAVE_REG         (*(reg32 *) mSPI_SCB__INTR_S)
#define mSPI_INTR_SLAVE_PTR         ( (reg32 *) mSPI_SCB__INTR_S)

#define mSPI_INTR_SLAVE_SET_REG     (*(reg32 *) mSPI_SCB__INTR_S_SET)
#define mSPI_INTR_SLAVE_SET_PTR     ( (reg32 *) mSPI_SCB__INTR_S_SET)

#define mSPI_INTR_SLAVE_MASK_REG    (*(reg32 *) mSPI_SCB__INTR_S_MASK)
#define mSPI_INTR_SLAVE_MASK_PTR    ( (reg32 *) mSPI_SCB__INTR_S_MASK)

#define mSPI_INTR_SLAVE_MASKED_REG  (*(reg32 *) mSPI_SCB__INTR_S_MASKED)
#define mSPI_INTR_SLAVE_MASKED_PTR  ( (reg32 *) mSPI_SCB__INTR_S_MASKED)

#define mSPI_INTR_TX_REG            (*(reg32 *) mSPI_SCB__INTR_TX)
#define mSPI_INTR_TX_PTR            ( (reg32 *) mSPI_SCB__INTR_TX)

#define mSPI_INTR_TX_SET_REG        (*(reg32 *) mSPI_SCB__INTR_TX_SET)
#define mSPI_INTR_TX_SET_PTR        ( (reg32 *) mSPI_SCB__INTR_TX_SET)

#define mSPI_INTR_TX_MASK_REG       (*(reg32 *) mSPI_SCB__INTR_TX_MASK)
#define mSPI_INTR_TX_MASK_PTR       ( (reg32 *) mSPI_SCB__INTR_TX_MASK)

#define mSPI_INTR_TX_MASKED_REG     (*(reg32 *) mSPI_SCB__INTR_TX_MASKED)
#define mSPI_INTR_TX_MASKED_PTR     ( (reg32 *) mSPI_SCB__INTR_TX_MASKED)

#define mSPI_INTR_RX_REG            (*(reg32 *) mSPI_SCB__INTR_RX)
#define mSPI_INTR_RX_PTR            ( (reg32 *) mSPI_SCB__INTR_RX)

#define mSPI_INTR_RX_SET_REG        (*(reg32 *) mSPI_SCB__INTR_RX_SET)
#define mSPI_INTR_RX_SET_PTR        ( (reg32 *) mSPI_SCB__INTR_RX_SET)

#define mSPI_INTR_RX_MASK_REG       (*(reg32 *) mSPI_SCB__INTR_RX_MASK)
#define mSPI_INTR_RX_MASK_PTR       ( (reg32 *) mSPI_SCB__INTR_RX_MASK)

#define mSPI_INTR_RX_MASKED_REG     (*(reg32 *) mSPI_SCB__INTR_RX_MASKED)
#define mSPI_INTR_RX_MASKED_PTR     ( (reg32 *) mSPI_SCB__INTR_RX_MASKED)

/* Defines get from SCB IP parameters. */
#define mSPI_FIFO_SIZE      (8u)  /* TX or RX FIFO size. */
#define mSPI_EZ_DATA_NR     (32u)  /* Number of words in EZ memory. */ 
#define mSPI_ONE_BYTE_WIDTH (8u)            /* Number of bits in one byte. */
#define mSPI_FF_DATA_NR_LOG2_MASK       (0x0Fu)      /* Number of bits to represent a FIFO address. */
#define mSPI_FF_DATA_NR_LOG2_PLUS1_MASK (0x1Fu) /* Number of bits to represent #bytes in FIFO. */


/***************************************
*        Registers Constants
***************************************/

#if (mSPI_SCB_IRQ_INTERNAL)
    #define mSPI_ISR_NUMBER     ((uint8) mSPI_SCB_IRQ__INTC_NUMBER)
    #define mSPI_ISR_PRIORITY   ((uint8) mSPI_SCB_IRQ__INTC_PRIOR_NUM)
#endif /* (mSPI_SCB_IRQ_INTERNAL) */

#if (mSPI_UART_RX_WAKEUP_IRQ)
    #define mSPI_RX_WAKE_ISR_NUMBER     ((uint8) mSPI_RX_WAKEUP_IRQ__INTC_NUMBER)
    #define mSPI_RX_WAKE_ISR_PRIORITY   ((uint8) mSPI_RX_WAKEUP_IRQ__INTC_PRIOR_NUM)
#endif /* (mSPI_UART_RX_WAKEUP_IRQ) */

/* mSPI_CTRL_REG */
#define mSPI_CTRL_OVS_POS           (0u)  /* [3:0]   Oversampling factor                 */
#define mSPI_CTRL_EC_AM_MODE_POS    (8u)  /* [8]     Externally clocked address match    */
#define mSPI_CTRL_EC_OP_MODE_POS    (9u)  /* [9]     Externally clocked operation mode   */
#define mSPI_CTRL_EZBUF_MODE_POS    (10u) /* [10]    EZ buffer is enabled                */
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_CTRL_BYTE_MODE_POS (11u) /* [11]    Determines the number of bits per FIFO data element */
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
#define mSPI_CTRL_ADDR_ACCEPT_POS   (16u) /* [16]    Put matched address in RX FIFO       */
#define mSPI_CTRL_BLOCK_POS         (17u) /* [17]    Ext and Int logic to resolve collide */
#define mSPI_CTRL_MODE_POS          (24u) /* [25:24] Operation mode                       */
#define mSPI_CTRL_ENABLED_POS       (31u) /* [31]    Enable SCB block                     */
#define mSPI_CTRL_OVS_MASK          ((uint32) 0x0Fu)
#define mSPI_CTRL_EC_AM_MODE        ((uint32) 0x01u << mSPI_CTRL_EC_AM_MODE_POS)
#define mSPI_CTRL_EC_OP_MODE        ((uint32) 0x01u << mSPI_CTRL_EC_OP_MODE_POS)
#define mSPI_CTRL_EZBUF_MODE        ((uint32) 0x01u << mSPI_CTRL_EZBUF_MODE_POS)
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_CTRL_BYTE_MODE ((uint32) 0x01u << mSPI_CTRL_BYTE_MODE_POS)
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
#define mSPI_CTRL_ADDR_ACCEPT       ((uint32) 0x01u << mSPI_CTRL_ADDR_ACCEPT_POS)
#define mSPI_CTRL_BLOCK             ((uint32) 0x01u << mSPI_CTRL_BLOCK_POS)
#define mSPI_CTRL_MODE_MASK         ((uint32) 0x03u << mSPI_CTRL_MODE_POS)
#define mSPI_CTRL_MODE_I2C          ((uint32) 0x00u)
#define mSPI_CTRL_MODE_SPI          ((uint32) 0x01u << mSPI_CTRL_MODE_POS)
#define mSPI_CTRL_MODE_UART         ((uint32) 0x02u << mSPI_CTRL_MODE_POS)
#define mSPI_CTRL_ENABLED           ((uint32) 0x01u << mSPI_CTRL_ENABLED_POS)

/* mSPI_STATUS_REG */
#define mSPI_STATUS_EC_BUSY_POS     (0u)  /* [0] Bus busy. Externally clocked logic access to EZ memory */
#define mSPI_STATUS_EC_BUSY         ((uint32) 0x0Fu)

/* mSPI_SPI_CTRL_REG  */
#define mSPI_SPI_CTRL_CONTINUOUS_POS        (0u)  /* [0]     Continuous or Separated SPI data transfers */
#define mSPI_SPI_CTRL_SELECT_PRECEDE_POS    (1u)  /* [1]     Precedes or coincides start of data frame  */
#define mSPI_SPI_CTRL_CPHA_POS              (2u)  /* [2]     SCLK phase                                 */
#define mSPI_SPI_CTRL_CPOL_POS              (3u)  /* [3]     SCLK polarity                              */
#define mSPI_SPI_CTRL_LATE_MISO_SAMPLE_POS  (4u)  /* [4]     Late MISO sample enabled                   */
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_SPI_CTRL_SCLK_CONTINUOUS_POS   (5u)  /* [5]     Enable continuous SCLK generation */
    #define mSPI_SPI_CTRL_SSEL0_POLARITY_POS    (8u)  /* [8]     SS0 polarity                      */
    #define mSPI_SPI_CTRL_SSEL1_POLARITY_POS    (9u)  /* [9]     SS1 polarity                      */
    #define mSPI_SPI_CTRL_SSEL2_POLARITY_POS    (10u) /* [10]    SS2 polarity                      */
    #define mSPI_SPI_CTRL_SSEL3_POLARITY_POS    (11u) /* [11]    SS3 polarity                      */
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */
#define mSPI_SPI_CTRL_LOOPBACK_POS          (16u) /* [16]    Local loop-back control enabled            */
#define mSPI_SPI_CTRL_MODE_POS              (24u) /* [25:24] Submode of SPI operation                   */
#define mSPI_SPI_CTRL_SLAVE_SELECT_POS      (26u) /* [27:26] Selects SPI SS signal                      */
#define mSPI_SPI_CTRL_MASTER_MODE_POS       (31u) /* [31]    Master mode enabled                        */
#define mSPI_SPI_CTRL_CONTINUOUS            ((uint32) 0x01u)
#define mSPI_SPI_CTRL_SELECT_PRECEDE        ((uint32) 0x01u << mSPI_SPI_CTRL_SELECT_PRECEDE_POS)
#define mSPI_SPI_CTRL_SCLK_MODE_MASK        ((uint32) 0x03u << mSPI_SPI_CTRL_CPHA_POS)
#define mSPI_SPI_CTRL_CPHA                  ((uint32) 0x01u << mSPI_SPI_CTRL_CPHA_POS)
#define mSPI_SPI_CTRL_CPOL                  ((uint32) 0x01u << mSPI_SPI_CTRL_CPOL_POS)
#define mSPI_SPI_CTRL_LATE_MISO_SAMPLE      ((uint32) 0x01u << \
                                                                    mSPI_SPI_CTRL_LATE_MISO_SAMPLE_POS)
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_SPI_CTRL_SCLK_CONTINUOUS  ((uint32) 0x01u << mSPI_SPI_CTRL_SCLK_CONTINUOUS_POS)
    #define mSPI_SPI_CTRL_SSEL0_POLARITY   ((uint32) 0x01u << mSPI_SPI_CTRL_SSEL0_POLARITY_POS)
    #define mSPI_SPI_CTRL_SSEL1_POLARITY   ((uint32) 0x01u << mSPI_SPI_CTRL_SSEL1_POLARITY_POS)
    #define mSPI_SPI_CTRL_SSEL2_POLARITY   ((uint32) 0x01u << mSPI_SPI_CTRL_SSEL2_POLARITY_POS)
    #define mSPI_SPI_CTRL_SSEL3_POLARITY   ((uint32) 0x01u << mSPI_SPI_CTRL_SSEL3_POLARITY_POS)
    #define mSPI_SPI_CTRL_SSEL_POLARITY_MASK ((uint32)0x0Fu << mSPI_SPI_CTRL_SSEL0_POLARITY_POS)
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

#define mSPI_SPI_CTRL_LOOPBACK              ((uint32) 0x01u << mSPI_SPI_CTRL_LOOPBACK_POS)
#define mSPI_SPI_CTRL_MODE_MASK             ((uint32) 0x03u << mSPI_SPI_CTRL_MODE_POS)
#define mSPI_SPI_CTRL_MODE_MOTOROLA         ((uint32) 0x00u)
#define mSPI_SPI_CTRL_MODE_TI               ((uint32) 0x01u << mSPI_CTRL_MODE_POS)
#define mSPI_SPI_CTRL_MODE_NS               ((uint32) 0x02u << mSPI_CTRL_MODE_POS)
#define mSPI_SPI_CTRL_SLAVE_SELECT_MASK     ((uint32) 0x03u << mSPI_SPI_CTRL_SLAVE_SELECT_POS)
#define mSPI_SPI_CTRL_SLAVE_SELECT0         ((uint32) 0x00u)
#define mSPI_SPI_CTRL_SLAVE_SELECT1         ((uint32) 0x01u << mSPI_SPI_CTRL_SLAVE_SELECT_POS)
#define mSPI_SPI_CTRL_SLAVE_SELECT2         ((uint32) 0x02u << mSPI_SPI_CTRL_SLAVE_SELECT_POS)
#define mSPI_SPI_CTRL_SLAVE_SELECT3         ((uint32) 0x03u << mSPI_SPI_CTRL_SLAVE_SELECT_POS)
#define mSPI_SPI_CTRL_MASTER                ((uint32) 0x01u << mSPI_SPI_CTRL_MASTER_MODE_POS)
#define mSPI_SPI_CTRL_SLAVE                 ((uint32) 0x00u)

/* mSPI_SPI_STATUS_REG  */
#define mSPI_SPI_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy - slave selected */
#define mSPI_SPI_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EzAddress                 */
#define mSPI_SPI_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define mSPI_SPI_STATUS_EZBUF_ADDR_MASK ((uint32) 0xFFu << mSPI_I2C_STATUS_EZBUF_ADDR_POS)

/* mSPI_UART_CTRL */
#define mSPI_UART_CTRL_LOOPBACK_POS         (16u) /* [16] Loop-back    */
#define mSPI_UART_CTRL_MODE_POS             (24u) /* [24] UART subMode */
#define mSPI_UART_CTRL_LOOPBACK             ((uint32) 0x01u << mSPI_UART_CTRL_LOOPBACK_POS)
#define mSPI_UART_CTRL_MODE_UART_STD        ((uint32) 0x00u)
#define mSPI_UART_CTRL_MODE_UART_SMARTCARD  ((uint32) 0x01u << mSPI_UART_CTRL_MODE_POS)
#define mSPI_UART_CTRL_MODE_UART_IRDA       ((uint32) 0x02u << mSPI_UART_CTRL_MODE_POS)
#define mSPI_UART_CTRL_MODE_MASK            ((uint32) 0x03u << mSPI_UART_CTRL_MODE_POS)

/* mSPI_UART_TX_CTRL */
#define mSPI_UART_TX_CTRL_STOP_BITS_POS         (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 period */
#define mSPI_UART_TX_CTRL_PARITY_POS            (4u)  /* [4]   Parity bit                              */
#define mSPI_UART_TX_CTRL_PARITY_ENABLED_POS    (5u)  /* [5]   Parity enable                           */
#define mSPI_UART_TX_CTRL_RETRY_ON_NACK_POS     (8u)  /* [8]   Smart Card: re-send frame on NACK       */
#define mSPI_UART_TX_CTRL_ONE_STOP_BIT          ((uint32) 0x01u)
#define mSPI_UART_TX_CTRL_ONE_HALF_STOP_BITS    ((uint32) 0x02u)
#define mSPI_UART_TX_CTRL_TWO_STOP_BITS         ((uint32) 0x03u)
#define mSPI_UART_TX_CTRL_STOP_BITS_MASK        ((uint32) 0x07u)
#define mSPI_UART_TX_CTRL_PARITY                ((uint32) 0x01u << \
                                                                    mSPI_UART_TX_CTRL_PARITY_POS)
#define mSPI_UART_TX_CTRL_PARITY_ENABLED        ((uint32) 0x01u << \
                                                                    mSPI_UART_TX_CTRL_PARITY_ENABLED_POS)
#define mSPI_UART_TX_CTRL_RETRY_ON_NACK         ((uint32) 0x01u << \
                                                                    mSPI_UART_TX_CTRL_RETRY_ON_NACK_POS)

/* mSPI_UART_RX_CTRL */
#define mSPI_UART_RX_CTRL_STOP_BITS_POS             (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 period*/
#define mSPI_UART_RX_CTRL_PARITY_POS                (4u)  /* [4]   Parity bit                             */
#define mSPI_UART_RX_CTRL_PARITY_ENABLED_POS        (5u)  /* [5]   Parity enable                          */
#define mSPI_UART_RX_CTRL_POLARITY_POS              (6u)  /* [6]   IrDA: inverts polarity of RX signal    */
#define mSPI_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS    (8u)  /* [8]   Drop and lost RX FIFO on parity error  */
#define mSPI_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS     (9u)  /* [9]   Drop and lost RX FIFO on frame error   */
#define mSPI_UART_RX_CTRL_MP_MODE_POS               (10u) /* [10]  Multi-processor mode                   */
#define mSPI_UART_RX_CTRL_LIN_MODE_POS              (12u) /* [12]  Lin mode: applicable for UART Standard */
#define mSPI_UART_RX_CTRL_SKIP_START_POS            (13u) /* [13]  Skip start not: only for UART Standard */
#define mSPI_UART_RX_CTRL_BREAK_WIDTH_POS           (16u) /* [19:16]  Break width: (Break width + 1)      */
#define mSPI_UART_TX_CTRL_ONE_STOP_BIT              ((uint32) 0x01u)
#define mSPI_UART_TX_CTRL_ONE_HALF_STOP_BITS        ((uint32) 0x02u)
#define mSPI_UART_TX_CTRL_TWO_STOP_BITS             ((uint32) 0x03u)
#define mSPI_UART_RX_CTRL_STOP_BITS_MASK            ((uint32) 0x07u)
#define mSPI_UART_RX_CTRL_PARITY                    ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_PARITY_POS)
#define mSPI_UART_RX_CTRL_PARITY_ENABLED            ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_PARITY_ENABLED_POS)
#define mSPI_UART_RX_CTRL_POLARITY                  ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_POLARITY_POS)
#define mSPI_UART_RX_CTRL_DROP_ON_PARITY_ERR        ((uint32) 0x01u << \
                                                                   mSPI_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS)
#define mSPI_UART_RX_CTRL_DROP_ON_FRAME_ERR         ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS)
#define mSPI_UART_RX_CTRL_MP_MODE                   ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_MP_MODE_POS)
#define mSPI_UART_RX_CTRL_LIN_MODE                  ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_LIN_MODE_POS)
#define mSPI_UART_RX_CTRL_SKIP_START                ((uint32) 0x01u << \
                                                                    mSPI_UART_RX_CTRL_SKIP_START_POS)
#define mSPI_UART_RX_CTRL_BREAK_WIDTH_MASK          ((uint32) 0x0Fu << \
                                                                    mSPI_UART_RX_CTRL_BREAK_WIDTH_POS)
/* mSPI_UART_RX_STATUS_REG */
#define mSPI_UART_RX_STATUS_BR_COUNTER_POS     (0u)  /* [11:0] Baud Rate counter */
#define mSPI_UART_RX_STATUS_BR_COUNTER_MASK    ((uint32) 0xFFFu)

#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    /* mSPI_UART_FLOW_CTRL_REG */
    #define mSPI_UART_FLOW_CTRL_TRIGGER_LEVEL_POS    (0u)  /* [7:0] RTS RX FIFO trigger level         */
    #define mSPI_UART_FLOW_CTRL_RTS_POLARITY_POS     (16u) /* [16]  Polarity of the RTS output signal */
    #define mSPI_UART_FLOW_CTRL_CTS_POLARITY_POS     (24u) /* [24]  Polarity of the CTS input signal  */
    #define mSPI_UART_FLOW_CTRL_CTS_ENABLED_POS      (25u) /* [25]  Enable CTS signal                 */
    #define mSPI_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK   ((uint32) mSPI_FF_DATA_NR_LOG2_MASK)
    #define mSPI_UART_FLOW_CTRL_RTS_POLARITY         ((uint32) 0x01u << \
                                                                       mSPI_UART_FLOW_CTRL_RTS_POLARITY_POS)
    #define mSPI_UART_FLOW_CTRL_CTS_POLARITY         ((uint32) 0x01u << \
                                                                       mSPI_UART_FLOW_CTRL_CTS_POLARITY_POS)
    #define mSPI_UART_FLOW_CTRL_CTS_ENABLE           ((uint32) 0x01u << \
                                                                       mSPI_UART_FLOW_CTRL_CTS_ENABLED_POS)
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

/* mSPI_I2C_CTRL */
#define mSPI_I2C_CTRL_HIGH_PHASE_OVS_POS           (0u)   /* [3:0] Oversampling factor high: master only */
#define mSPI_I2C_CTRL_LOW_PHASE_OVS_POS            (4u)   /* [7:4] Oversampling factor low:  master only */
#define mSPI_I2C_CTRL_M_READY_DATA_ACK_POS         (8u)   /* [8]   Master ACKs data while RX FIFO != FULL*/
#define mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK_POS    (9u)   /* [9]   Master NACKs data if RX FIFO ==  FULL */
#define mSPI_I2C_CTRL_S_GENERAL_IGNORE_POS         (11u)  /* [11]  Slave ignores General call            */
#define mSPI_I2C_CTRL_S_READY_ADDR_ACK_POS         (12u)  /* [12]  Slave ACKs Address if RX FIFO != FULL */
#define mSPI_I2C_CTRL_S_READY_DATA_ACK_POS         (13u)  /* [13]  Slave ACKs data while RX FIFO == FULL */
#define mSPI_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS    (14u)  /* [14]  Slave NACKs address if RX FIFO == FULL*/
#define mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK_POS    (15u)  /* [15]  Slave NACKs data if RX FIFO is  FULL  */
#define mSPI_I2C_CTRL_LOOPBACK_POS                 (16u)  /* [16]  Loop-back                             */
#define mSPI_I2C_CTRL_SLAVE_MODE_POS               (30u)  /* [30]  Slave mode enabled                    */
#define mSPI_I2C_CTRL_MASTER_MODE_POS              (31u)  /* [31]  Master mode enabled                   */
#define mSPI_I2C_CTRL_HIGH_PHASE_OVS_MASK  ((uint32) 0x0Fu)
#define mSPI_I2C_CTRL_LOW_PHASE_OVS_MASK   ((uint32) 0x0Fu << \
                                                                mSPI_I2C_CTRL_LOW_PHASE_OVS_POS)
#define mSPI_I2C_CTRL_M_READY_DATA_ACK      ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_M_READY_DATA_ACK_POS)
#define mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK_POS)
#define mSPI_I2C_CTRL_S_GENERAL_IGNORE      ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_S_GENERAL_IGNORE_POS)
#define mSPI_I2C_CTRL_S_READY_ADDR_ACK      ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_S_READY_ADDR_ACK_POS)
#define mSPI_I2C_CTRL_S_READY_DATA_ACK      ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_S_READY_DATA_ACK_POS)
#define mSPI_I2C_CTRL_S_NOT_READY_ADDR_NACK ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS)
#define mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK_POS)
#define mSPI_I2C_CTRL_LOOPBACK              ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_LOOPBACK_POS)
#define mSPI_I2C_CTRL_SLAVE_MODE            ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_SLAVE_MODE_POS)
#define mSPI_I2C_CTRL_MASTER_MODE           ((uint32) 0x01u << \
                                                                mSPI_I2C_CTRL_MASTER_MODE_POS)
#define mSPI_I2C_CTRL_SLAVE_MASTER_MODE_MASK    ((uint32) 0x03u << \
                                                                mSPI_I2C_CTRL_SLAVE_MODE_POS)

/* mSPI_I2C_STATUS_REG  */
#define mSPI_I2C_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy: internally clocked */
#define mSPI_I2C_STATUS_S_READ_POS      (4u)  /* [4]    Slave is read by master      */
#define mSPI_I2C_STATUS_M_READ_POS      (5u)  /* [5]    Master reads Slave           */
#define mSPI_I2C_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EZAddress                    */
#define mSPI_I2C_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define mSPI_I2C_STATUS_S_READ          ((uint32) 0x01u << mSPI_I2C_STATUS_S_READ_POS)
#define mSPI_I2C_STATUS_M_READ          ((uint32) 0x01u << mSPI_I2C_STATUS_M_READ_POS)
#define mSPI_I2C_STATUS_EZBUF_ADDR_MASK ((uint32) 0xFFu << mSPI_I2C_STATUS_EZBUF_ADDR_POS)

/* mSPI_I2C_MASTER_CMD_REG */
#define mSPI_I2C_MASTER_CMD_M_START_POS             (0u)  /* [0] Master generate Start                */
#define mSPI_I2C_MASTER_CMD_M_START_ON_IDLE_POS     (1u)  /* [1] Master generate Start if bus is free */
#define mSPI_I2C_MASTER_CMD_M_ACK_POS               (2u)  /* [2] Master generate ACK                  */
#define mSPI_I2C_MASTER_CMD_M_NACK_POS              (3u)  /* [3] Master generate NACK                 */
#define mSPI_I2C_MASTER_CMD_M_STOP_POS              (4u)  /* [4] Master generate Stop                 */
#define mSPI_I2C_MASTER_CMD_M_START         ((uint32) 0x01u)
#define mSPI_I2C_MASTER_CMD_M_START_ON_IDLE ((uint32) 0x01u << \
                                                                   mSPI_I2C_MASTER_CMD_M_START_ON_IDLE_POS)
#define mSPI_I2C_MASTER_CMD_M_ACK           ((uint32) 0x01u << \
                                                                   mSPI_I2C_MASTER_CMD_M_ACK_POS)
#define mSPI_I2C_MASTER_CMD_M_NACK          ((uint32) 0x01u << \
                                                                    mSPI_I2C_MASTER_CMD_M_NACK_POS)
#define mSPI_I2C_MASTER_CMD_M_STOP          ((uint32) 0x01u << \
                                                                    mSPI_I2C_MASTER_CMD_M_STOP_POS)

/* mSPI_I2C_SLAVE_CMD_REG  */
#define mSPI_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define mSPI_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define mSPI_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define mSPI_I2C_SLAVE_CMD_S_NACK       ((uint32) 0x01u << mSPI_I2C_SLAVE_CMD_S_NACK_POS)

#define mSPI_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define mSPI_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define mSPI_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define mSPI_I2C_SLAVE_CMD_S_NACK       ((uint32) 0x01u << mSPI_I2C_SLAVE_CMD_S_NACK_POS)

/* mSPI_I2C_CFG_REG */
#if (mSPI_CY_SCBIP_V0)
#define mSPI_I2C_CFG_SDA_FILT_HYS_POS           (0u)  /* [1:0]   Trim bits for the I2C SDA filter         */
#define mSPI_I2C_CFG_SDA_FILT_TRIM_POS          (2u)  /* [3:2]   Trim bits for the I2C SDA filter         */
#define mSPI_I2C_CFG_SCL_FILT_HYS_POS           (4u)  /* [5:4]   Trim bits for the I2C SCL filter         */
#define mSPI_I2C_CFG_SCL_FILT_TRIM_POS          (6u)  /* [7:6]   Trim bits for the I2C SCL filter         */
#define mSPI_I2C_CFG_SDA_FILT_OUT_HYS_POS       (8u)  /* [9:8]   Trim bits for I2C SDA filter output path */
#define mSPI_I2C_CFG_SDA_FILT_OUT_TRIM_POS      (10u) /* [11:10] Trim bits for I2C SDA filter output path */
#define mSPI_I2C_CFG_SDA_FILT_HS_POS            (16u) /* [16]    '0': 50 ns filter, '1': 10 ns filter     */
#define mSPI_I2C_CFG_SDA_FILT_ENABLED_POS       (17u) /* [17]    I2C SDA filter enabled                   */
#define mSPI_I2C_CFG_SCL_FILT_HS_POS            (24u) /* [24]    '0': 50 ns filter, '1': 10 ns filter     */
#define mSPI_I2C_CFG_SCL_FILT_ENABLED_POS       (25u) /* [25]    I2C SCL filter enabled                   */
#define mSPI_I2C_CFG_SDA_FILT_OUT_HS_POS        (26u) /* [26]    '0': 50 ns filter, '1': 10 ns filter     */
#define mSPI_I2C_CFG_SDA_FILT_OUT_ENABLED_POS   (27u) /* [27]    I2C SDA output delay filter enabled      */
#define mSPI_I2C_CFG_SDA_FILT_HYS_MASK          ((uint32) 0x03u)
#define mSPI_I2C_CFG_SDA_FILT_TRIM_MASK         ((uint32) 0x03u << \
                                                                mSPI_I2C_CFG_SDA_FILT_TRIM_POS)
#define mSPI_I2C_CFG_SCL_FILT_HYS_MASK          ((uint32) 0x03u << \
                                                                mSPI_I2C_CFG_SCL_FILT_HYS_POS)
#define mSPI_I2C_CFG_SCL_FILT_TRIM_MASK         ((uint32) 0x03u << \
                                                                mSPI_I2C_CFG_SCL_FILT_TRIM_POS)
#define mSPI_I2C_CFG_SDA_FILT_OUT_HYS_MASK      ((uint32) 0x03u << \
                                                                mSPI_I2C_CFG_SDA_FILT_OUT_HYS_POS)
#define mSPI_I2C_CFG_SDA_FILT_OUT_TRIM_MASK     ((uint32) 0x03u << \
                                                                mSPI_I2C_CFG_SDA_FILT_OUT_TRIM_POS)
#define mSPI_I2C_CFG_SDA_FILT_HS                ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SDA_FILT_HS_POS)
#define mSPI_I2C_CFG_SDA_FILT_ENABLED           ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SDA_FILT_ENABLED_POS)
#define mSPI_I2C_CFG_SCL_FILT_HS                ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SCL_FILT_HS_POS)
#define mSPI_I2C_CFG_SCL_FILT_ENABLED           ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SCL_FILT_ENABLED_POS)
#define mSPI_I2C_CFG_SDA_FILT_OUT_HS            ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SDA_FILT_OUT_HS_POS)
#define mSPI_I2C_CFG_SDA_FILT_OUT_ENABLED       ((uint32) 0x01u << \
                                                                mSPI_I2C_CFG_SDA_FILT_OUT_ENABLED_POS)
#else
#define mSPI_I2C_CFG_SDA_IN_FILT_TRIM_POS   (0u)  /* [1:0] Trim bits for "i2c_sda_in" 50 ns filter */
#define mSPI_I2C_CFG_SDA_IN_FILT_SEL_POS    (4u)  /* [4]   "i2c_sda_in" filter delay: 0 ns and 50 ns */
#define mSPI_I2C_CFG_SCL_IN_FILT_TRIM_POS   (8u)  /* [9:8] Trim bits for "i2c_scl_in" 50 ns filter */
#define mSPI_I2C_CFG_SCL_IN_FILT_SEL_POS    (12u) /* [12]  "i2c_scl_in" filter delay: 0 ns and 50 ns */
#define mSPI_I2C_CFG_SDA_OUT_FILT0_TRIM_POS (16u) /* [17:16] Trim bits for "i2c_sda_out" 50 ns filter 0 */
#define mSPI_I2C_CFG_SDA_OUT_FILT1_TRIM_POS (18u) /* [19:18] Trim bits for "i2c_sda_out" 50 ns filter 1 */
#define mSPI_I2C_CFG_SDA_OUT_FILT2_TRIM_POS (20u) /* [21:20] Trim bits for "i2c_sda_out" 50 ns filter 2 */
#define mSPI_I2C_CFG_SDA_OUT_FILT_SEL_POS   (28u) /* [29:28] Cumulative "i2c_sda_out" filter delay: */

#define mSPI_I2C_CFG_SDA_IN_FILT_TRIM_MASK  ((uint32) 0x03u)
#define mSPI_I2C_CFG_SDA_IN_FILT_SEL        ((uint32) 0x01u << mSPI_I2C_CFG_SDA_IN_FILT_SEL_POS)
#define mSPI_I2C_CFG_SCL_IN_FILT_TRIM_MASK  ((uint32) 0x03u << \
                                                            mSPI_I2C_CFG_SCL_IN_FILT_TRIM_POS)
#define mSPI_I2C_CFG_SCL_IN_FILT_SEL        ((uint32) 0x01u << mSPI_I2C_CFG_SCL_IN_FILT_SEL_POS)
#define mSPI_I2C_CFG_SDA_OUT_FILT0_TRIM_MASK ((uint32) 0x03u << \
                                                            mSPI_I2C_CFG_SDA_OUT_FILT0_TRIM_POS)
#define mSPI_I2C_CFG_SDA_OUT_FILT1_TRIM_MASK ((uint32) 0x03u << \
                                                            mSPI_I2C_CFG_SDA_OUT_FILT1_TRIM_POS)
#define mSPI_I2C_CFG_SDA_OUT_FILT2_TRIM_MASK ((uint32) 0x03u << \
                                                            mSPI_I2C_CFG_SDA_OUT_FILT2_TRIM_POS)
#define mSPI_I2C_CFG_SDA_OUT_FILT_SEL_MASK   ((uint32) 0x03u << \
                                                            mSPI_I2C_CFG_SDA_OUT_FILT_SEL_POS)
#endif /* (mSPI_CY_SCBIP_V0) */


/* mSPI_TX_CTRL_REG */
#define mSPI_TX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Data frame width: (Data width - 1) */
#define mSPI_TX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define mSPI_TX_CTRL_ENABLED_POS        (31u) /* [31]  Transmitter enabled               */
#define mSPI_TX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define mSPI_TX_CTRL_MSB_FIRST          ((uint32) 0x01u << mSPI_TX_CTRL_MSB_FIRST_POS)
#define mSPI_TX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define mSPI_TX_CTRL_ENABLED            ((uint32) 0x01u << mSPI_TX_CTRL_ENABLED_POS)

/* mSPI_TX_CTRL_FIFO_REG */
#define mSPI_TX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)  /* [2:0] Trigger level                              */
#define mSPI_TX_FIFO_CTRL_CLEAR_POS             (16u) /* [16]  Clear TX FIFO: cleared after set           */
#define mSPI_TX_FIFO_CTRL_FREEZE_POS            (17u) /* [17]  Freeze TX FIFO: HW do not inc read pointer */
#define mSPI_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) mSPI_FF_DATA_NR_LOG2_MASK)
#define mSPI_TX_FIFO_CTRL_CLEAR                 ((uint32) 0x01u << mSPI_TX_FIFO_CTRL_CLEAR_POS)
#define mSPI_TX_FIFO_CTRL_FREEZE                ((uint32) 0x01u << mSPI_TX_FIFO_CTRL_FREEZE_POS)

/* mSPI_TX_FIFO_STATUS_REG */
#define mSPI_TX_FIFO_STATUS_USED_POS    (0u)  /* [3:0]   Amount of entries in TX FIFO */
#define mSPI_TX_FIFO_SR_VALID_POS       (15u) /* [15]    Shifter status of TX FIFO    */
#define mSPI_TX_FIFO_STATUS_RD_PTR_POS  (16u) /* [18:16] TX FIFO read pointer         */
#define mSPI_TX_FIFO_STATUS_WR_PTR_POS  (24u) /* [26:24] TX FIFO write pointer        */
#define mSPI_TX_FIFO_STATUS_USED_MASK   ((uint32) mSPI_FF_DATA_NR_LOG2_PLUS1_MASK)
#define mSPI_TX_FIFO_SR_VALID           ((uint32) 0x01u << mSPI_TX_FIFO_SR_VALID_POS)
#define mSPI_TX_FIFO_STATUS_RD_PTR_MASK ((uint32) mSPI_FF_DATA_NR_LOG2_MASK << \
                                                                    mSPI_TX_FIFO_STATUS_RD_PTR_POS)
#define mSPI_TX_FIFO_STATUS_WR_PTR_MASK ((uint32) mSPI_FF_DATA_NR_LOG2_MASK << \
                                                                    mSPI_TX_FIFO_STATUS_WR_PTR_POS)

/* mSPI_TX_FIFO_WR_REG */
#define mSPI_TX_FIFO_WR_POS    (0u)  /* [15:0] Data written into TX FIFO */
#define mSPI_TX_FIFO_WR_MASK   ((uint32) 0xFFu)

/* mSPI_RX_CTRL_REG */
#define mSPI_RX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Data frame width: (Data width - 1) */
#define mSPI_RX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define mSPI_RX_CTRL_MEDIAN_POS         (9u)  /* [9]   Median filter                     */
#define mSPI_RX_CTRL_ENABLED_POS        (31u) /* [31]  Receiver enabled                  */
#define mSPI_RX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define mSPI_RX_CTRL_MSB_FIRST          ((uint32) 0x01u << mSPI_RX_CTRL_MSB_FIRST_POS)
#define mSPI_RX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define mSPI_RX_CTRL_MEDIAN             ((uint32) 0x01u << mSPI_RX_CTRL_MEDIAN_POS)
#define mSPI_RX_CTRL_ENABLED            ((uint32) 0x01u << mSPI_RX_CTRL_ENABLED_POS)


/* mSPI_RX_FIFO_CTRL_REG */
#define mSPI_RX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)   /* [2:0] Trigger level                            */
#define mSPI_RX_FIFO_CTRL_CLEAR_POS             (16u)  /* [16]  Clear RX FIFO: clear after set           */
#define mSPI_RX_FIFO_CTRL_FREEZE_POS            (17u)  /* [17]  Freeze RX FIFO: HW writes has not effect */
#define mSPI_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) mSPI_FF_DATA_NR_LOG2_MASK)
#define mSPI_RX_FIFO_CTRL_CLEAR                 ((uint32) 0x01u << mSPI_RX_FIFO_CTRL_CLEAR_POS)
#define mSPI_RX_FIFO_CTRL_FREEZE                ((uint32) 0x01u << mSPI_RX_FIFO_CTRL_FREEZE_POS)

/* mSPI_RX_FIFO_STATUS_REG */
#define mSPI_RX_FIFO_STATUS_USED_POS    (0u)   /* [3:0]   Amount of entries in RX FIFO */
#define mSPI_RX_FIFO_SR_VALID_POS       (15u)  /* [15]    Shifter status of RX FIFO    */
#define mSPI_RX_FIFO_STATUS_RD_PTR_POS  (16u)  /* [18:16] RX FIFO read pointer         */
#define mSPI_RX_FIFO_STATUS_WR_PTR_POS  (24u)  /* [26:24] RX FIFO write pointer        */
#define mSPI_RX_FIFO_STATUS_USED_MASK   ((uint32) mSPI_FF_DATA_NR_LOG2_PLUS1_MASK)
#define mSPI_RX_FIFO_SR_VALID           ((uint32) 0x01u << mSPI_RX_FIFO_SR_VALID_POS)
#define mSPI_RX_FIFO_STATUS_RD_PTR_MASK ((uint32) mSPI_FF_DATA_NR_LOG2_MASK << \
                                                                    mSPI_RX_FIFO_STATUS_RD_PTR_POS)
#define mSPI_RX_FIFO_STATUS_WR_PTR_MASK ((uint32) mSPI_FF_DATA_NR_LOG2_MASK << \
                                                                    mSPI_RX_FIFO_STATUS_WR_PTR_POS)

/* mSPI_RX_MATCH_REG */
#define mSPI_RX_MATCH_ADDR_POS     (0u)  /* [7:0]   Slave address                        */
#define mSPI_RX_MATCH_MASK_POS     (16u) /* [23:16] Slave address mask: 0 - doesn't care */
#define mSPI_RX_MATCH_ADDR_MASK    ((uint32) 0xFFu)
#define mSPI_RX_MATCH_MASK_MASK    ((uint32) 0xFFu << mSPI_RX_MATCH_MASK_POS)

/* mSPI_RX_FIFO_WR_REG */
#define mSPI_RX_FIFO_RD_POS    (0u)  /* [15:0] Data read from RX FIFO */
#define mSPI_RX_FIFO_RD_MASK   ((uint32) 0xFFu)

/* mSPI_RX_FIFO_RD_SILENT_REG */
#define mSPI_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define mSPI_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* mSPI_RX_FIFO_RD_SILENT_REG */
#define mSPI_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define mSPI_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* mSPI_EZBUF_DATA_REG */
#define mSPI_EZBUF_DATA_POS   (0u)  /* [7:0] Data from EZ Memory */
#define mSPI_EZBUF_DATA_MASK  ((uint32) 0xFFu)

/*  mSPI_INTR_CAUSE_REG */
#define mSPI_INTR_CAUSE_MASTER_POS  (0u)  /* [0] Master interrupt active                 */
#define mSPI_INTR_CAUSE_SLAVE_POS   (1u)  /* [1] Slave interrupt active                  */
#define mSPI_INTR_CAUSE_TX_POS      (2u)  /* [2] Transmitter interrupt active            */
#define mSPI_INTR_CAUSE_RX_POS      (3u)  /* [3] Receiver interrupt active               */
#define mSPI_INTR_CAUSE_I2C_EC_POS  (4u)  /* [4] Externally clock I2C interrupt active   */
#define mSPI_INTR_CAUSE_SPI_EC_POS  (5u)  /* [5] Externally clocked SPI interrupt active */
#define mSPI_INTR_CAUSE_MASTER      ((uint32) 0x01u)
#define mSPI_INTR_CAUSE_SLAVE       ((uint32) 0x01u << mSPI_INTR_CAUSE_SLAVE_POS)
#define mSPI_INTR_CAUSE_TX          ((uint32) 0x01u << mSPI_INTR_CAUSE_TX_POS)
#define mSPI_INTR_CAUSE_RX          ((uint32) 0x01u << mSPI_INTR_CAUSE_RX_POS)
#define mSPI_INTR_CAUSE_I2C_EC      ((uint32) 0x01u << mSPI_INTR_CAUSE_I2C_EC_POS)
#define mSPI_INTR_CAUSE_SPI_EC      ((uint32) 0x01u << mSPI_INTR_CAUSE_SPI_EC_POS)

/* mSPI_INTR_SPI_EC_REG, mSPI_INTR_SPI_EC_MASK_REG, mSPI_INTR_SPI_EC_MASKED_REG */
#define mSPI_INTR_SPI_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define mSPI_INTR_SPI_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define mSPI_INTR_SPI_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define mSPI_INTR_SPI_EC_WAKE_UP              ((uint32) 0x01u)
#define mSPI_INTR_SPI_EC_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    mSPI_INTR_SPI_EC_EZBUF_STOP_POS)
#define mSPI_INTR_SPI_EC_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                    mSPI_INTR_SPI_EC_EZBUF_WRITE_STOP_POS)

/* mSPI_INTR_I2C_EC, mSPI_INTR_I2C_EC_MASK, mSPI_INTR_I2C_EC_MASKED */
#define mSPI_INTR_I2C_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define mSPI_INTR_I2C_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define mSPI_INTR_I2C_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define mSPI_INTR_I2C_EC_WAKE_UP              ((uint32) 0x01u)
#define mSPI_INTR_I2C_EC_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    mSPI_INTR_I2C_EC_EZBUF_STOP_POS)
#define mSPI_INTR_I2C_EC_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                    mSPI_INTR_I2C_EC_EZBUF_WRITE_STOP_POS)

/* mSPI_INTR_MASTER, mSPI_INTR_MASTER_SET,
   mSPI_INTR_MASTER_MASK, mSPI_INTR_MASTER_MASKED */
#define mSPI_INTR_MASTER_I2C_ARB_LOST_POS   (0u)  /* [0] Master lost arbitration                          */
#define mSPI_INTR_MASTER_I2C_NACK_POS       (1u)  /* [1] Master receives NACK: address or write to slave  */
#define mSPI_INTR_MASTER_I2C_ACK_POS        (2u)  /* [2] Master receives NACK: address or write to slave  */
#define mSPI_INTR_MASTER_I2C_STOP_POS       (4u)  /* [4] Master detects the Stop: only self generated Stop*/
#define mSPI_INTR_MASTER_I2C_BUS_ERROR_POS  (8u)  /* [8] Master detects bus error: misplaced Start or Stop*/
#define mSPI_INTR_MASTER_SPI_DONE_POS       (9u)  /* [9] Master complete transfer: Only for SPI           */
#define mSPI_INTR_MASTER_I2C_ARB_LOST       ((uint32) 0x01u)
#define mSPI_INTR_MASTER_I2C_NACK           ((uint32) 0x01u << mSPI_INTR_MASTER_I2C_NACK_POS)
#define mSPI_INTR_MASTER_I2C_ACK            ((uint32) 0x01u << mSPI_INTR_MASTER_I2C_ACK_POS)
#define mSPI_INTR_MASTER_I2C_STOP           ((uint32) 0x01u << mSPI_INTR_MASTER_I2C_STOP_POS)
#define mSPI_INTR_MASTER_I2C_BUS_ERROR      ((uint32) 0x01u << \
                                                                    mSPI_INTR_MASTER_I2C_BUS_ERROR_POS)
#define mSPI_INTR_MASTER_SPI_DONE           ((uint32) 0x01u << mSPI_INTR_MASTER_SPI_DONE_POS)

/*
* mSPI_INTR_SLAVE, mSPI_INTR_SLAVE_SET,
* mSPI_INTR_SLAVE_MASK, mSPI_INTR_SLAVE_MASKED
*/
#define mSPI_INTR_SLAVE_I2C_ARB_LOST_POS         (0u)  /* [0]  Slave lost arbitration                   */
#define mSPI_INTR_SLAVE_I2C_NACK_POS             (1u)  /* [1]  Slave receives NACK: master reads data   */
#define mSPI_INTR_SLAVE_I2C_ACK_POS              (2u)  /* [2]  Slave receives ACK: master reads data    */
#define mSPI_INTR_SLAVE_I2C_WRITE_STOP_POS       (3u)  /* [3]  Slave detects end of write transaction   */
#define mSPI_INTR_SLAVE_I2C_STOP_POS             (4u)  /* [4]  Slave detects end of transaction intended */
#define mSPI_INTR_SLAVE_I2C_START_POS            (5u)  /* [5]  Slave detects Start                      */
#define mSPI_INTR_SLAVE_I2C_ADDR_MATCH_POS       (6u)  /* [6]  Slave address matches                    */
#define mSPI_INTR_SLAVE_I2C_GENERAL_POS          (7u)  /* [7]  General call received                    */
#define mSPI_INTR_SLAVE_I2C_BUS_ERROR_POS        (8u)  /* [8]  Slave detects bus error                  */
#define mSPI_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS (9u)  /* [9]  Slave write complete: Only for SPI       */
#define mSPI_INTR_SLAVE_SPI_EZBUF_STOP_POS       (10u) /* [10] Slave end of transaction: Only for SPI   */
#define mSPI_INTR_SLAVE_SPI_BUS_ERROR_POS        (11u) /* [11] Slave detects bus error: Only for SPI    */
#define mSPI_INTR_SLAVE_I2C_ARB_LOST             ((uint32) 0x01u)
#define mSPI_INTR_SLAVE_I2C_NACK                 ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_NACK_POS)
#define mSPI_INTR_SLAVE_I2C_ACK                  ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_ACK_POS)
#define mSPI_INTR_SLAVE_I2C_WRITE_STOP           ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_WRITE_STOP_POS)
#define mSPI_INTR_SLAVE_I2C_STOP                 ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_STOP_POS)
#define mSPI_INTR_SLAVE_I2C_START                ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_START_POS)
#define mSPI_INTR_SLAVE_I2C_ADDR_MATCH           ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_ADDR_MATCH_POS)
#define mSPI_INTR_SLAVE_I2C_GENERAL              ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_GENERAL_POS)
#define mSPI_INTR_SLAVE_I2C_BUS_ERROR            ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_I2C_BUS_ERROR_POS)
#define mSPI_INTR_SLAVE_SPI_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                   mSPI_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS)
#define mSPI_INTR_SLAVE_SPI_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_SPI_EZBUF_STOP_POS)
#define mSPI_INTR_SLAVE_SPI_BUS_ERROR           ((uint32) 0x01u << \
                                                                    mSPI_INTR_SLAVE_SPI_BUS_ERROR_POS)

/*
* mSPI_INTR_TX, mSPI_INTR_TX_SET,
* mSPI_INTR_TX_MASK, mSPI_INTR_TX_MASKED
*/
#define mSPI_INTR_TX_TRIGGER_POS        (0u)  /* [0]  Trigger on TX FIFO entires                       */
#define mSPI_INTR_TX_NOT_FULL_POS       (1u)  /* [1]  TX FIFO is not full                              */
#define mSPI_INTR_TX_EMPTY_POS          (4u)  /* [4]  TX FIFO is empty                                 */
#define mSPI_INTR_TX_OVERFLOW_POS       (5u)  /* [5]  Attempt to write to a full TX FIFO               */
#define mSPI_INTR_TX_UNDERFLOW_POS      (6u)  /* [6]  Attempt to read from an empty TX FIFO            */
#define mSPI_INTR_TX_BLOCKED_POS        (7u)  /* [7]  No access to the EZ memory                       */
#define mSPI_INTR_TX_UART_NACK_POS      (8u)  /* [8]  UART transmitter received a NACK: SmartCard mode */
#define mSPI_INTR_TX_UART_DONE_POS      (9u)  /* [9]  UART transmitter done even                       */
#define mSPI_INTR_TX_UART_ARB_LOST_POS  (10u) /* [10] UART lost arbitration: LIN or SmartCard          */
#define mSPI_INTR_TX_TRIGGER            ((uint32) 0x01u)
#define mSPI_INTR_TX_FIFO_LEVEL         (mSPI_INTR_TX_TRIGGER)
#define mSPI_INTR_TX_NOT_FULL           ((uint32) 0x01u << mSPI_INTR_TX_NOT_FULL_POS)
#define mSPI_INTR_TX_EMPTY              ((uint32) 0x01u << mSPI_INTR_TX_EMPTY_POS)
#define mSPI_INTR_TX_OVERFLOW           ((uint32) 0x01u << mSPI_INTR_TX_OVERFLOW_POS)
#define mSPI_INTR_TX_UNDERFLOW          ((uint32) 0x01u << mSPI_INTR_TX_UNDERFLOW_POS)
#define mSPI_INTR_TX_BLOCKED            ((uint32) 0x01u << mSPI_INTR_TX_BLOCKED_POS)
#define mSPI_INTR_TX_UART_NACK          ((uint32) 0x01u << mSPI_INTR_TX_UART_NACK_POS)
#define mSPI_INTR_TX_UART_DONE          ((uint32) 0x01u << mSPI_INTR_TX_UART_DONE_POS)
#define mSPI_INTR_TX_UART_ARB_LOST      ((uint32) 0x01u << mSPI_INTR_TX_UART_ARB_LOST_POS)

/*
* mSPI_INTR_RX, mSPI_INTR_RX_SET,
* mSPI_INTR_RX_MASK, mSPI_INTR_RX_MASKED
*/
#define mSPI_INTR_RX_TRIGGER_POS        (0u)   /* [0]  Trigger on RX FIFO entires            */
#define mSPI_INTR_RX_NOT_EMPTY_POS      (2u)   /* [2]  RX FIFO is not empty                  */
#define mSPI_INTR_RX_FULL_POS           (3u)   /* [3]  RX FIFO is full                       */
#define mSPI_INTR_RX_OVERFLOW_POS       (5u)   /* [5]  Attempt to write to a full RX FIFO    */
#define mSPI_INTR_RX_UNDERFLOW_POS      (6u)   /* [6]  Attempt to read from an empty RX FIFO */
#define mSPI_INTR_RX_BLOCKED_POS        (7u)   /* [7]  No access to the EZ memory            */
#define mSPI_INTR_RX_FRAME_ERROR_POS    (8u)   /* [8]  Frame error in received data frame    */
#define mSPI_INTR_RX_PARITY_ERROR_POS   (9u)   /* [9]  Parity error in received data frame   */
#define mSPI_INTR_RX_BAUD_DETECT_POS    (10u)  /* [10] LIN baud rate detection is completed   */
#define mSPI_INTR_RX_BREAK_DETECT_POS   (11u)  /* [11] Break detection is successful         */
#define mSPI_INTR_RX_TRIGGER            ((uint32) 0x01u)
#define mSPI_INTR_RX_FIFO_LEVEL         (mSPI_INTR_RX_TRIGGER)
#define mSPI_INTR_RX_NOT_EMPTY          ((uint32) 0x01u << mSPI_INTR_RX_NOT_EMPTY_POS)
#define mSPI_INTR_RX_FULL               ((uint32) 0x01u << mSPI_INTR_RX_FULL_POS)
#define mSPI_INTR_RX_OVERFLOW           ((uint32) 0x01u << mSPI_INTR_RX_OVERFLOW_POS)
#define mSPI_INTR_RX_UNDERFLOW          ((uint32) 0x01u << mSPI_INTR_RX_UNDERFLOW_POS)
#define mSPI_INTR_RX_BLOCKED            ((uint32) 0x01u << mSPI_INTR_RX_BLOCKED_POS)
#define mSPI_INTR_RX_FRAME_ERROR        ((uint32) 0x01u << mSPI_INTR_RX_FRAME_ERROR_POS)
#define mSPI_INTR_RX_PARITY_ERROR       ((uint32) 0x01u << mSPI_INTR_RX_PARITY_ERROR_POS)
#define mSPI_INTR_RX_BAUD_DETECT        ((uint32) 0x01u << mSPI_INTR_RX_BAUD_DETECT_POS)
#define mSPI_INTR_RX_BREAK_DETECT       ((uint32) 0x01u << mSPI_INTR_RX_BREAK_DETECT_POS)

/* Define all interrupt sources */
#define mSPI_INTR_I2C_EC_ALL    (mSPI_INTR_I2C_EC_WAKE_UP    | \
                                             mSPI_INTR_I2C_EC_EZBUF_STOP | \
                                             mSPI_INTR_I2C_EC_EZBUF_WRITE_STOP)

#define mSPI_INTR_SPI_EC_ALL    (mSPI_INTR_SPI_EC_WAKE_UP    | \
                                             mSPI_INTR_SPI_EC_EZBUF_STOP | \
                                             mSPI_INTR_SPI_EC_EZBUF_WRITE_STOP)

#define mSPI_INTR_MASTER_ALL    (mSPI_INTR_MASTER_I2C_ARB_LOST  | \
                                             mSPI_INTR_MASTER_I2C_NACK      | \
                                             mSPI_INTR_MASTER_I2C_ACK       | \
                                             mSPI_INTR_MASTER_I2C_STOP      | \
                                             mSPI_INTR_MASTER_I2C_BUS_ERROR | \
                                             mSPI_INTR_MASTER_SPI_DONE)

#define mSPI_INTR_SLAVE_ALL     (mSPI_INTR_SLAVE_I2C_ARB_LOST      | \
                                             mSPI_INTR_SLAVE_I2C_NACK          | \
                                             mSPI_INTR_SLAVE_I2C_ACK           | \
                                             mSPI_INTR_SLAVE_I2C_WRITE_STOP    | \
                                             mSPI_INTR_SLAVE_I2C_STOP          | \
                                             mSPI_INTR_SLAVE_I2C_START         | \
                                             mSPI_INTR_SLAVE_I2C_ADDR_MATCH    | \
                                             mSPI_INTR_SLAVE_I2C_GENERAL       | \
                                             mSPI_INTR_SLAVE_I2C_BUS_ERROR     | \
                                             mSPI_INTR_SLAVE_SPI_EZBUF_WRITE_STOP | \
                                             mSPI_INTR_SLAVE_SPI_EZBUF_STOP       | \
                                             mSPI_INTR_SLAVE_SPI_BUS_ERROR)

#define mSPI_INTR_TX_ALL        (mSPI_INTR_TX_TRIGGER   | \
                                             mSPI_INTR_TX_NOT_FULL  | \
                                             mSPI_INTR_TX_EMPTY     | \
                                             mSPI_INTR_TX_OVERFLOW  | \
                                             mSPI_INTR_TX_UNDERFLOW | \
                                             mSPI_INTR_TX_BLOCKED   | \
                                             mSPI_INTR_TX_UART_NACK | \
                                             mSPI_INTR_TX_UART_DONE | \
                                             mSPI_INTR_TX_UART_ARB_LOST)

#define mSPI_INTR_RX_ALL        (mSPI_INTR_RX_TRIGGER      | \
                                             mSPI_INTR_RX_NOT_EMPTY    | \
                                             mSPI_INTR_RX_FULL         | \
                                             mSPI_INTR_RX_OVERFLOW     | \
                                             mSPI_INTR_RX_UNDERFLOW    | \
                                             mSPI_INTR_RX_BLOCKED      | \
                                             mSPI_INTR_RX_FRAME_ERROR  | \
                                             mSPI_INTR_RX_PARITY_ERROR | \
                                             mSPI_INTR_RX_BAUD_DETECT  | \
                                             mSPI_INTR_RX_BREAK_DETECT)

/* I2C and EZI2C slave address defines */
#define mSPI_I2C_SLAVE_ADDR_POS    (0x01u)    /* 7-bit address shift */
#define mSPI_I2C_SLAVE_ADDR_MASK   (0xFEu)    /* 8-bit address mask */

/* OVS constants for IrDA Low Power operation */
#define mSPI_CTRL_OVS_IRDA_LP_OVS16     (0x00u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS32     (0x01u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS48     (0x02u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS96     (0x03u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS192    (0x04u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS768    (0x05u)
#define mSPI_CTRL_OVS_IRDA_LP_OVS1536   (0x06u)

/* OVS constant for IrDA */
#define mSPI_CTRL_OVS_IRDA_OVS16        (mSPI_UART_IRDA_LP_OVS16)


/***************************************
*    Common Macro Definitions
***************************************/

/* Re-enables the SCB IP. A clear enable bit has a different effect
* on the scb IP depending on the version:
*  CY_SCBIP_V0: resets state, status, TX and RX FIFOs.
*  CY_SCBIP_V1 or later: resets state, status, TX and RX FIFOs and interrupt sources.
* Clear I2C command registers are because they are not impacted by re-enable.
*/
#define mSPI_SCB_SW_RESET   mSPI_I2CFwBlockReset()

/* TX FIFO macro */
#define mSPI_CLEAR_TX_FIFO \
                            do{        \
                                mSPI_TX_FIFO_CTRL_REG |= ((uint32)  mSPI_TX_FIFO_CTRL_CLEAR); \
                                mSPI_TX_FIFO_CTRL_REG &= ((uint32) ~mSPI_TX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define mSPI_GET_TX_FIFO_ENTRIES    (mSPI_TX_FIFO_STATUS_REG & \
                                                 mSPI_TX_FIFO_STATUS_USED_MASK)

#define mSPI_GET_TX_FIFO_SR_VALID   ((0u != (mSPI_TX_FIFO_STATUS_REG & \
                                                         mSPI_TX_FIFO_SR_VALID)) ? (1u) : (0u))

/* RX FIFO macro */
#define mSPI_CLEAR_RX_FIFO \
                            do{        \
                                mSPI_RX_FIFO_CTRL_REG |= ((uint32)  mSPI_RX_FIFO_CTRL_CLEAR); \
                                mSPI_RX_FIFO_CTRL_REG &= ((uint32) ~mSPI_RX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define mSPI_GET_RX_FIFO_ENTRIES    (mSPI_RX_FIFO_STATUS_REG & \
                                                    mSPI_RX_FIFO_STATUS_USED_MASK)

#define mSPI_GET_RX_FIFO_SR_VALID   ((0u != (mSPI_RX_FIFO_STATUS_REG & \
                                                         mSPI_RX_FIFO_SR_VALID)) ? (1u) : (0u))

/* Write interrupt source: set sourceMask bits in mSPI_INTR_X_MASK_REG */
#define mSPI_WRITE_INTR_I2C_EC_MASK(sourceMask) \
                                                do{         \
                                                    mSPI_INTR_I2C_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_WRITE_INTR_SPI_EC_MASK(sourceMask) \
                                                do{         \
                                                    mSPI_INTR_SPI_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_WRITE_INTR_MASTER_MASK(sourceMask) \
                                                do{         \
                                                    mSPI_INTR_MASTER_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_WRITE_INTR_SLAVE_MASK(sourceMask)  \
                                                do{         \
                                                    mSPI_INTR_SLAVE_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_WRITE_INTR_TX_MASK(sourceMask)     \
                                                do{         \
                                                    mSPI_INTR_TX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_WRITE_INTR_RX_MASK(sourceMask)     \
                                                do{         \
                                                    mSPI_INTR_RX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Enable interrupt source: set sourceMask bits in mSPI_INTR_X_MASK_REG */
#define mSPI_ENABLE_INTR_I2C_EC(sourceMask) \
                                                do{     \
                                                    mSPI_INTR_I2C_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_ENABLE_INTR_SPI_EC(sourceMask) \
                                                do{     \
                                                    mSPI_INTR_SPI_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_ENABLE_INTR_MASTER(sourceMask) \
                                                do{     \
                                                    mSPI_INTR_MASTER_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_ENABLE_INTR_SLAVE(sourceMask)  \
                                                do{     \
                                                    mSPI_INTR_SLAVE_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_ENABLE_INTR_TX(sourceMask)     \
                                                do{     \
                                                    mSPI_INTR_TX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_ENABLE_INTR_RX(sourceMask)     \
                                                do{     \
                                                    mSPI_INTR_RX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

/* Disable interrupt source: clear sourceMask bits in mSPI_INTR_X_MASK_REG */
#define mSPI_DISABLE_INTR_I2C_EC(sourceMask) \
                                do{                      \
                                    mSPI_INTR_I2C_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_DISABLE_INTR_SPI_EC(sourceMask) \
                                do{                      \
                                    mSPI_INTR_SPI_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_DISABLE_INTR_MASTER(sourceMask) \
                                do{                      \
                                mSPI_INTR_MASTER_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define mSPI_DISABLE_INTR_SLAVE(sourceMask) \
                                do{                     \
                                    mSPI_INTR_SLAVE_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define mSPI_DISABLE_INTR_TX(sourceMask)    \
                                do{                     \
                                    mSPI_INTR_TX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)

#define mSPI_DISABLE_INTR_RX(sourceMask)    \
                                do{                     \
                                    mSPI_INTR_RX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

/* Set interrupt sources: write sourceMask bits in mSPI_INTR_X_SET_REG */
#define mSPI_SET_INTR_MASTER(sourceMask)    \
                                                do{     \
                                                    mSPI_INTR_MASTER_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_SET_INTR_SLAVE(sourceMask) \
                                                do{ \
                                                    mSPI_INTR_SLAVE_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_SET_INTR_TX(sourceMask)    \
                                                do{ \
                                                    mSPI_INTR_TX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_SET_INTR_RX(sourceMask)    \
                                                do{ \
                                                    mSPI_INTR_RX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Clear interrupt sources: write sourceMask bits in mSPI_INTR_X_REG */
#define mSPI_CLEAR_INTR_I2C_EC(sourceMask)  \
                                                do{     \
                                                    mSPI_INTR_I2C_EC_REG = (uint32) (sourceMask); \
                                                }while(0)

#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_CLEAR_INTR_SPI_EC(sourceMask)  \
                                                do{     \
                                                    mSPI_INTR_SPI_EC_REG = (uint32) (sourceMask); \
                                                }while(0)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_CLEAR_INTR_MASTER(sourceMask)  \
                                                do{     \
                                                    mSPI_INTR_MASTER_REG = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_CLEAR_INTR_SLAVE(sourceMask)   \
                                                do{     \
                                                    mSPI_INTR_SLAVE_REG  = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_CLEAR_INTR_TX(sourceMask)      \
                                                do{     \
                                                    mSPI_INTR_TX_REG     = (uint32) (sourceMask); \
                                                }while(0)

#define mSPI_CLEAR_INTR_RX(sourceMask)      \
                                                do{     \
                                                    mSPI_INTR_RX_REG     = (uint32) (sourceMask); \
                                                }while(0)

/* Return true if sourceMask is set in mSPI_INTR_CAUSE_REG */
#define mSPI_CHECK_CAUSE_INTR(sourceMask)    (0u != (mSPI_INTR_CAUSE_REG & (sourceMask)))

/* Return true if sourceMask is set in INTR_X_MASKED_REG */
#define mSPI_CHECK_INTR_I2C_EC(sourceMask)  (0u != (mSPI_INTR_I2C_EC_REG & (sourceMask)))
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_CHECK_INTR_SPI_EC(sourceMask)  (0u != (mSPI_INTR_SPI_EC_REG & (sourceMask)))
#endif /* (!mSPI_CY_SCBIP_V1) */
#define mSPI_CHECK_INTR_MASTER(sourceMask)  (0u != (mSPI_INTR_MASTER_REG & (sourceMask)))
#define mSPI_CHECK_INTR_SLAVE(sourceMask)   (0u != (mSPI_INTR_SLAVE_REG  & (sourceMask)))
#define mSPI_CHECK_INTR_TX(sourceMask)      (0u != (mSPI_INTR_TX_REG     & (sourceMask)))
#define mSPI_CHECK_INTR_RX(sourceMask)      (0u != (mSPI_INTR_RX_REG     & (sourceMask)))

/* Return true if sourceMask is set in mSPI_INTR_X_MASKED_REG */
#define mSPI_CHECK_INTR_I2C_EC_MASKED(sourceMask)   (0u != (mSPI_INTR_I2C_EC_MASKED_REG & \
                                                                       (sourceMask)))
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_CHECK_INTR_SPI_EC_MASKED(sourceMask)   (0u != (mSPI_INTR_SPI_EC_MASKED_REG & \
                                                                       (sourceMask)))
#endif /* (!mSPI_CY_SCBIP_V1) */
#define mSPI_CHECK_INTR_MASTER_MASKED(sourceMask)   (0u != (mSPI_INTR_MASTER_MASKED_REG & \
                                                                       (sourceMask)))
#define mSPI_CHECK_INTR_SLAVE_MASKED(sourceMask)    (0u != (mSPI_INTR_SLAVE_MASKED_REG  & \
                                                                       (sourceMask)))
#define mSPI_CHECK_INTR_TX_MASKED(sourceMask)       (0u != (mSPI_INTR_TX_MASKED_REG     & \
                                                                       (sourceMask)))
#define mSPI_CHECK_INTR_RX_MASKED(sourceMask)       (0u != (mSPI_INTR_RX_MASKED_REG     & \
                                                                       (sourceMask)))

/* Return true if sourceMask is set in mSPI_CTRL_REG: generally is used to check enable bit */
#define mSPI_GET_CTRL_ENABLED    (0u != (mSPI_CTRL_REG & mSPI_CTRL_ENABLED))

#define mSPI_CHECK_SLAVE_AUTO_ADDR_NACK     (0u != (mSPI_I2C_CTRL_REG & \
                                                                mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK))


/***************************************
*      I2C Macro Definitions
***************************************/

/* Enable auto ACK/NACK */
#define mSPI_ENABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                      \
                                mSPI_I2C_CTRL_REG |= mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define mSPI_ENABLE_SLAVE_AUTO_DATA_ACK \
                            do{                     \
                                mSPI_I2C_CTRL_REG |= mSPI_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define mSPI_ENABLE_SLAVE_AUTO_DATA_NACK \
                            do{                      \
                                mSPI_I2C_CTRL_REG |= mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define mSPI_ENABLE_MASTER_AUTO_DATA_ACK \
                            do{                      \
                                mSPI_I2C_CTRL_REG |= mSPI_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define mSPI_ENABLE_MASTER_AUTO_DATA_NACK \
                            do{                       \
                                mSPI_I2C_CTRL_REG |= mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Disable auto ACK/NACK */
#define mSPI_DISABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                       \
                                mSPI_I2C_CTRL_REG &= ~mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define mSPI_DISABLE_SLAVE_AUTO_DATA_ACK \
                            do{                      \
                                mSPI_I2C_CTRL_REG &= ~mSPI_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define mSPI_DISABLE_SLAVE_AUTO_DATA_NACK \
                            do{                       \
                                mSPI_I2C_CTRL_REG &= ~mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define mSPI_DISABLE_MASTER_AUTO_DATA_ACK \
                            do{                       \
                                mSPI_I2C_CTRL_REG &= ~mSPI_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define mSPI_DISABLE_MASTER_AUTO_DATA_NACK \
                            do{                        \
                                mSPI_I2C_CTRL_REG &= ~mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Enable Slave autoACK/NACK Data */
#define mSPI_ENABLE_SLAVE_AUTO_DATA \
                            do{                 \
                                mSPI_I2C_CTRL_REG |= (mSPI_I2C_CTRL_S_READY_DATA_ACK |      \
                                                                  mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK); \
                            }while(0)

/* Disable Slave autoACK/NACK Data */
#define mSPI_DISABLE_SLAVE_AUTO_DATA \
                            do{                  \
                                mSPI_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(mSPI_I2C_CTRL_S_READY_DATA_ACK |       \
                                                                    mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK)); \
                            }while(0)

/* Disable Master autoACK/NACK Data */
#define mSPI_DISABLE_MASTER_AUTO_DATA \
                            do{                   \
                                mSPI_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(mSPI_I2C_CTRL_M_READY_DATA_ACK |       \
                                                                    mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK)); \
                            }while(0)
/* Disables auto data ACK/NACK bits */
#define mSPI_DISABLE_AUTO_DATA \
                do{                        \
                    mSPI_I2C_CTRL_REG &= ((uint32) ~(mSPI_I2C_CTRL_M_READY_DATA_ACK      |  \
                                                                 mSPI_I2C_CTRL_M_NOT_READY_DATA_NACK |  \
                                                                 mSPI_I2C_CTRL_S_READY_DATA_ACK      |  \
                                                                 mSPI_I2C_CTRL_S_NOT_READY_DATA_NACK)); \
                }while(0)

/* Master commands */
#define mSPI_I2C_MASTER_GENERATE_START \
                            do{                    \
                                mSPI_I2C_MASTER_CMD_REG = mSPI_I2C_MASTER_CMD_M_START_ON_IDLE; \
                            }while(0)

#define mSPI_I2C_MASTER_CLEAR_START \
                            do{                 \
                                mSPI_I2C_MASTER_CMD_REG =  ((uint32) 0u); \
                            }while(0)

#define mSPI_I2C_MASTER_GENERATE_RESTART mSPI_I2CReStartGeneration()

#define mSPI_I2C_MASTER_GENERATE_STOP \
                            do{                   \
                                mSPI_I2C_MASTER_CMD_REG =                                            \
                                    (mSPI_I2C_MASTER_CMD_M_STOP |                                    \
                                        (mSPI_CHECK_I2C_STATUS(mSPI_I2C_STATUS_M_READ) ? \
                                            (mSPI_I2C_MASTER_CMD_M_NACK) : (0u)));                   \
                            }while(0)

#define mSPI_I2C_MASTER_GENERATE_ACK \
                            do{                  \
                                mSPI_I2C_MASTER_CMD_REG = mSPI_I2C_MASTER_CMD_M_ACK; \
                            }while(0)

#define mSPI_I2C_MASTER_GENERATE_NACK \
                            do{                   \
                                mSPI_I2C_MASTER_CMD_REG = mSPI_I2C_MASTER_CMD_M_NACK; \
                            }while(0)

/* Slave commands */
#define mSPI_I2C_SLAVE_GENERATE_ACK \
                            do{                 \
                                mSPI_I2C_SLAVE_CMD_REG = mSPI_I2C_SLAVE_CMD_S_ACK; \
                            }while(0)

#if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    /* Slave NACK generation for EC_AM logic on address phase. Ticket ID #183902 */
    void mSPI_I2CSlaveNackGeneration(void);
    #define mSPI_I2C_SLAVE_GENERATE_NACK mSPI_I2CSlaveNackGeneration()

#else
    #define mSPI_I2C_SLAVE_GENERATE_NACK \
                            do{                      \
                                mSPI_I2C_SLAVE_CMD_REG = mSPI_I2C_SLAVE_CMD_S_NACK; \
                            }while(0)
#endif /* (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

#define mSPI_I2C_SLAVE_CLEAR_NACK \
                            do{               \
                                mSPI_I2C_SLAVE_CMD_REG = 0u; \
                            }while(0)

/* Return 8-bit address. The input address should be 7-bits */
#define mSPI_GET_I2C_8BIT_ADDRESS(addr) (((uint32) ((uint32) (addr) << \
                                                                    mSPI_I2C_SLAVE_ADDR_POS)) & \
                                                                        mSPI_I2C_SLAVE_ADDR_MASK)

#define mSPI_GET_I2C_7BIT_ADDRESS(addr) ((uint32) (addr) >> mSPI_I2C_SLAVE_ADDR_POS)

/* Adjust SDA filter Trim settings */
#define mSPI_DEFAULT_I2C_CFG_SDA_FILT_TRIM  (0x02u)
#define mSPI_EC_AM_I2C_CFG_SDA_FILT_TRIM    (0x03u)

#if (mSPI_CY_SCBIP_V0)
    #define mSPI_SET_I2C_CFG_SDA_FILT_TRIM(sdaTrim) \
        do{                                                 \
            mSPI_I2C_CFG_REG =                  \
                            ((mSPI_I2C_CFG_REG & (uint32) ~mSPI_I2C_CFG_SDA_FILT_TRIM_MASK) | \
                             ((uint32) ((uint32) (sdaTrim) <<mSPI_I2C_CFG_SDA_FILT_TRIM_POS)));           \
        }while(0)
#endif /* (mSPI_CY_SCBIP_V0) */

/* Enable/Disable analog and digital filter */
#define mSPI_DIGITAL_FILTER_DISABLE    (0u)
#define mSPI_DIGITAL_FILTER_ENABLE     (1u)
#define mSPI_I2C_DATA_RATE_FS_MODE_MAX (400u)
#if (mSPI_CY_SCBIP_V0)
    /* mSPI_I2C_CFG_SDA_FILT_OUT_ENABLED is disabled by default */
    #define mSPI_I2C_CFG_FILT_MASK  (mSPI_I2C_CFG_SDA_FILT_ENABLED | \
                                                 mSPI_I2C_CFG_SCL_FILT_ENABLED)
#else
    /* mSPI_I2C_CFG_SDA_OUT_FILT_SEL_MASK is disabled by default */
    #define mSPI_I2C_CFG_FILT_MASK  (mSPI_I2C_CFG_SDA_IN_FILT_SEL | \
                                                 mSPI_I2C_CFG_SCL_IN_FILT_SEL)
#endif /* (mSPI_CY_SCBIP_V0) */

#define mSPI_I2C_CFG_ANALOG_FITER_DISABLE \
        do{                                           \
            mSPI_I2C_CFG_REG &= (uint32) ~mSPI_I2C_CFG_FILT_MASK; \
        }while(0)

#define mSPI_I2C_CFG_ANALOG_FITER_ENABLE \
        do{                                          \
            mSPI_I2C_CFG_REG |= (uint32)  mSPI_I2C_CFG_FILT_MASK; \
        }while(0)

/* Return slave select number from SPI_CTRL register */
#define mSPI_GET_SPI_CTRL_SS(activeSelect) (((uint32) ((uint32) (activeSelect) << \
                                                                    mSPI_SPI_CTRL_SLAVE_SELECT_POS)) & \
                                                                        mSPI_SPI_CTRL_SLAVE_SELECT_MASK)

/* Return true if bit is set in mSPI_I2C_STATUS_REG */
#define mSPI_CHECK_I2C_STATUS(sourceMask)   (0u != (mSPI_I2C_STATUS_REG & (sourceMask)))

/* Return true if bit is set in mSPI_SPI_STATUS_REG */
#define mSPI_CHECK_SPI_STATUS(sourceMask)   (0u != (mSPI_SPI_STATUS_REG & (sourceMask)))

/* Return FIFO size depends on mSPI_CTRL_BYTE_MODE bit */
#define mSPI_GET_FIFO_SIZE(condition) ((0u != (condition)) ? \
                                                    (2u * mSPI_FIFO_SIZE) : (mSPI_FIFO_SIZE))


/***************************************
*       Get Macros Definitions
***************************************/

/* mSPI_CTRL */
#define mSPI_GET_CTRL_OVS(oversample)       (((uint32) (oversample) - 1u) & mSPI_CTRL_OVS_MASK)

#define mSPI_GET_CTRL_EC_OP_MODE(opMode)        ((0u != (opMode)) ? \
                                                                (mSPI_CTRL_EC_OP_MODE)  : (0u))

#define mSPI_GET_CTRL_EC_AM_MODE(amMode)        ((0u != (amMode)) ? \
                                                                (mSPI_CTRL_EC_AM_MODE)  : (0u))

#define mSPI_GET_CTRL_BLOCK(block)              ((0u != (block))  ? \
                                                                (mSPI_CTRL_BLOCK)       : (0u))

#define mSPI_GET_CTRL_ADDR_ACCEPT(acceptAddr)   ((0u != (acceptAddr)) ? \
                                                                (mSPI_CTRL_ADDR_ACCEPT) : (0u))

#if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_GET_CTRL_BYTE_MODE(mode)   (0u)
#else
    #define mSPI_GET_CTRL_BYTE_MODE(mode)   ((0u != (mode)) ? \
                                                            (mSPI_CTRL_BYTE_MODE) : (0u))
#endif /* (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

/* mSPI_I2C_CTRL */
#define mSPI_GET_I2C_CTRL_HIGH_PHASE_OVS(oversampleHigh) (((uint32) (oversampleHigh) - 1u) & \
                                                                        mSPI_I2C_CTRL_HIGH_PHASE_OVS_MASK)

#define mSPI_GET_I2C_CTRL_LOW_PHASE_OVS(oversampleLow)  ((((uint32) (oversampleLow) - 1u) << \
                                                                    mSPI_I2C_CTRL_LOW_PHASE_OVS_POS) &  \
                                                                    mSPI_I2C_CTRL_LOW_PHASE_OVS_MASK)

#define mSPI_GET_I2C_CTRL_S_NOT_READY_ADDR_NACK(wakeNack) ((0u != (wakeNack)) ? \
                                                            (mSPI_I2C_CTRL_S_NOT_READY_ADDR_NACK) : (0u))

#define mSPI_GET_I2C_CTRL_S_GENERAL_IGNORE(genCall) ((0u != (genCall)) ? \
                                                                    (mSPI_I2C_CTRL_S_GENERAL_IGNORE) : (0u))

#define mSPI_GET_I2C_CTRL_SL_MSTR_MODE(mode)    ((uint32)(mode) << mSPI_I2C_CTRL_SLAVE_MODE_POS)

/* mSPI_SPI_CTRL */
#define mSPI_GET_SPI_CTRL_CONTINUOUS(separate)  ((0u != (separate)) ? \
                                                                (mSPI_SPI_CTRL_CONTINUOUS) : (0u))

#define mSPI_GET_SPI_CTRL_SELECT_PRECEDE(mode)  ((0u != (mode)) ? \
                                                                      (mSPI_SPI_CTRL_SELECT_PRECEDE) : (0u))

#define mSPI_GET_SPI_CTRL_SCLK_MODE(mode)       (((uint32) (mode) << \
                                                                        mSPI_SPI_CTRL_CPHA_POS) & \
                                                                        mSPI_SPI_CTRL_SCLK_MODE_MASK)

#define mSPI_GET_SPI_CTRL_LATE_MISO_SAMPLE(lateMiso) ((0u != (lateMiso)) ? \
                                                                    (mSPI_SPI_CTRL_LATE_MISO_SAMPLE) : (0u))

#if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_GET_SPI_CTRL_SCLK_CONTINUOUS(sclkType) (0u)
    #define mSPI_GET_SPI_CTRL_SSEL_POLARITY(polarity)   (0u)
#else
    #define mSPI_GET_SPI_CTRL_SCLK_CONTINUOUS(sclkType) ((0u != (sclkType)) ? \
                                                                    (mSPI_SPI_CTRL_SCLK_CONTINUOUS) : (0u))

    #define mSPI_GET_SPI_CTRL_SSEL_POLARITY(polarity)   (((uint32) (polarity) << \
                                                                     mSPI_SPI_CTRL_SSEL0_POLARITY_POS) & \
                                                                     mSPI_SPI_CTRL_SSEL_POLARITY_MASK)
#endif /* ((mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

#define mSPI_GET_SPI_CTRL_SUB_MODE(mode)        (((uint32) (mode) << mSPI_SPI_CTRL_MODE_POS) & \
                                                                                 mSPI_SPI_CTRL_MODE_MASK)

#define mSPI_GET_SPI_CTRL_SLAVE_SELECT(select)  (((uint32) (select) << \
                                                                      mSPI_SPI_CTRL_SLAVE_SELECT_POS) & \
                                                                      mSPI_SPI_CTRL_SLAVE_SELECT_MASK)

#define mSPI_GET_SPI_CTRL_MASTER_MODE(mode)     ((0u != (mode)) ? \
                                                                (mSPI_SPI_CTRL_MASTER) : (0u))

/* mSPI_UART_CTRL */
#define mSPI_GET_UART_CTRL_MODE(mode)           (((uint32) (mode) << \
                                                                            mSPI_UART_CTRL_MODE_POS) & \
                                                                            mSPI_UART_CTRL_MODE_MASK)

/* mSPI_UART_RX_CTRL */
#define mSPI_GET_UART_RX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                        mSPI_UART_RX_CTRL_STOP_BITS_MASK)

#define mSPI_GET_UART_RX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                                    (mSPI_UART_RX_CTRL_PARITY) : (0u))

#define mSPI_GET_UART_RX_CTRL_POLARITY(polarity)    ((0u != (polarity)) ? \
                                                                    (mSPI_UART_RX_CTRL_POLARITY) : (0u))

#define mSPI_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (mSPI_UART_RX_CTRL_DROP_ON_PARITY_ERR) : (0u))

#define mSPI_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (mSPI_UART_RX_CTRL_DROP_ON_FRAME_ERR) : (0u))

#define mSPI_GET_UART_RX_CTRL_MP_MODE(mpMode)   ((0u != (mpMode)) ? \
                                                        (mSPI_UART_RX_CTRL_MP_MODE) : (0u))

#define mSPI_GET_UART_RX_CTRL_BREAK_WIDTH(width)    (((uint32) ((uint32) (width) - 1u) << \
                                                                    mSPI_UART_RX_CTRL_BREAK_WIDTH_POS) & \
                                                                    mSPI_UART_RX_CTRL_BREAK_WIDTH_MASK)

/* mSPI_UART_TX_CTRL */
#define mSPI_GET_UART_TX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                mSPI_UART_RX_CTRL_STOP_BITS_MASK)

#define mSPI_GET_UART_TX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                               (mSPI_UART_TX_CTRL_PARITY) : (0u))

#define mSPI_GET_UART_TX_CTRL_RETRY_NACK(nack)  ((0u != (nack)) ? \
                                                               (mSPI_UART_TX_CTRL_RETRY_ON_NACK) : (0u))

/* mSPI_UART_FLOW_CTRL */
#if !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
    #define mSPI_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(level)   ( (uint32) (level) & \
                                                                 mSPI_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK)

    #define mSPI_GET_UART_FLOW_CTRL_RTS_POLARITY(polarity) ((0u != (polarity)) ? \
                                                                (mSPI_UART_FLOW_CTRL_RTS_POLARITY) : (0u))

    #define mSPI_GET_UART_FLOW_CTRL_CTS_POLARITY(polarity) ((0u != (polarity)) ? \
                                                                (mSPI_UART_FLOW_CTRL_CTS_POLARITY) : (0u))

    #define mSPI_GET_UART_FLOW_CTRL_CTS_ENABLE(ctsEn)      ((0u != (ctsEn)) ? \
                                                                (mSPI_UART_FLOW_CTRL_CTS_ENABLE) : (0u))
#endif /* !(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

/* mSPI_RX_CTRL */
#define mSPI_GET_RX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                mSPI_RX_CTRL_DATA_WIDTH_MASK)

#define mSPI_GET_RX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (mSPI_RX_CTRL_MSB_FIRST) : (0u))

#define mSPI_GET_RX_CTRL_MEDIAN(filterEn)       ((0u != (filterEn)) ? \
                                                                (mSPI_RX_CTRL_MEDIAN) : (0u))

/* mSPI_RX_MATCH */
#define mSPI_GET_RX_MATCH_ADDR(addr)    ((uint32) (addr) & mSPI_RX_MATCH_ADDR_MASK)
#define mSPI_GET_RX_MATCH_MASK(mask)    (((uint32) (mask) << \
                                                            mSPI_RX_MATCH_MASK_POS) & \
                                                            mSPI_RX_MATCH_MASK_MASK)

/* mSPI_RX_FIFO_CTRL */
#define mSPI_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    mSPI_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* mSPI_TX_CTRL */
#define mSPI_GET_TX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                mSPI_TX_CTRL_DATA_WIDTH_MASK)

#define mSPI_GET_TX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (mSPI_TX_CTRL_MSB_FIRST) : (0u))

/* mSPI_TX_FIFO_CTRL */
#define mSPI_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    mSPI_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* mSPI_INTR_SLAVE_I2C_GENERAL */
#define mSPI_GET_INTR_SLAVE_I2C_GENERAL(genCall)  ((0u != (genCall)) ? \
                                                                (mSPI_INTR_SLAVE_I2C_GENERAL) : (0u))

/* Return true if master mode is enabled mSPI_SPI_CTRL_REG */
#define mSPI_CHECK_SPI_MASTER   (0u != (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_MASTER))

/* Return inactive state of SPI SCLK line depends on CPOL */
#define mSPI_GET_SPI_SCLK_INACTIVE \
            ((0u == (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_CPOL)) ? (0u) : (1u))

/* Get output pin inactive state */
#if (mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1)
#define mSPI_GET_SPI_SS0_INACTIVE       (1u)
#define mSPI_GET_SPI_SS1_INACTIVE       (1u)
#define mSPI_GET_SPI_SS2_INACTIVE       (1u)
#define mSPI_GET_SPI_SS3_INACTIVE       (1u)
#define mSPI_GET_UART_RTS_INACTIVE      (1u)

#else
#define mSPI_GET_SPI_SS0_INACTIVE  \
        ((0u != (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_SSEL0_POLARITY)) ? (0u) : (1u))

#define mSPI_GET_SPI_SS1_INACTIVE  \
        ((0u != (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_SSEL1_POLARITY)) ? (0u) : (1u))

#define mSPI_GET_SPI_SS2_INACTIVE  \
        ((0u != (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_SSEL2_POLARITY)) ? (0u) : (1u))

#define mSPI_GET_SPI_SS3_INACTIVE  \
        ((0u != (mSPI_SPI_CTRL_REG & mSPI_SPI_CTRL_SSEL3_POLARITY)) ? (0u) : (1u))

#define mSPI_GET_UART_RTS_INACTIVE \
        ((0u == (mSPI_UART_FLOW_CTRL_REG & mSPI_UART_FLOW_CTRL_RTS_POLARITY)) ? (0u) : (1u))

#endif /*(mSPI_CY_SCBIP_V0 || mSPI_CY_SCBIP_V1) */

/* Clear register constants for configuration and interrupt mask */
#define mSPI_CLEAR_REG          ((uint32) (0u))
#define mSPI_NO_INTR_SOURCES    ((uint32) (0u))
#define mSPI_DUMMY_PARAM        ((uint32) (0u))
#define mSPI_SUBMODE_SPI_SLAVE  ((uint32) (0u))

/* Return in case of I2C read error */
#define mSPI_I2C_INVALID_BYTE   ((uint32) 0xFFFFFFFFu)
#define mSPI_CHECK_VALID_BYTE   ((uint32) 0x80000000u)


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

#define mSPI_CHECK_INTR_EC_I2C(sourceMask)  mSPI_CHECK_INTR_I2C_EC(sourceMask)
#if (!mSPI_CY_SCBIP_V1)
    #define mSPI_CHECK_INTR_EC_SPI(sourceMask)  mSPI_CHECK_INTR_SPI_EC(sourceMask)
#endif /* (!mSPI_CY_SCBIP_V1) */

#define mSPI_CY_SCBIP_V1_I2C_ONLY   (mSPI_CY_SCBIP_V1)
#define mSPI_EZBUFFER_SIZE          (mSPI_EZ_DATA_NR)

#define mSPI_EZBUF_DATA00_REG   mSPI_EZBUF_DATA0_REG
#define mSPI_EZBUF_DATA00_PTR   mSPI_EZBUF_DATA0_PTR

#endif /* (CY_SCB_mSPI_H) */


/* [] END OF FILE */
