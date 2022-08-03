/***************************************************************************//**
* \file mUART_SPI_UART_INT.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
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

#include "mUART_PVT.h"
#include "mUART_SPI_UART_PVT.h"
#include "cyapicallbacks.h"

#if (mUART_SCB_IRQ_INTERNAL)
/*******************************************************************************
* Function Name: mUART_SPI_UART_ISR
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
*******************************************************************************/
CY_ISR(mUART_SPI_UART_ISR)
{
#if (mUART_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
#endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

#if (mUART_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */

#ifdef mUART_SPI_UART_ISR_ENTRY_CALLBACK
    mUART_SPI_UART_ISR_EntryCallback();
#endif /* mUART_SPI_UART_ISR_ENTRY_CALLBACK */

    if (NULL != mUART_customIntrHandler)
    {
        mUART_customIntrHandler();
    }

    #if(mUART_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        mUART_ClearSpiExtClkInterruptSource(mUART_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if (mUART_CHECK_RX_SW_BUFFER)
    {
        if (mUART_CHECK_INTR_RX_MASKED(mUART_INTR_RX_NOT_EMPTY))
        {
            do
            {
                /* Move local head index */
                locHead = (mUART_rxBufferHead + 1u);

                /* Adjust local head index */
                if (mUART_INTERNAL_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if (locHead == mUART_rxBufferTail)
                {
                    #if (mUART_CHECK_UART_RTS_CONTROL_FLOW)
                    {
                        /* There is no space in the software buffer - disable the
                        * RX Not Empty interrupt source. The data elements are
                        * still being received into the RX FIFO until the RTS signal
                        * stops the transmitter. After the data element is read from the
                        * buffer, the RX Not Empty interrupt source is enabled to
                        * move the next data element in the software buffer.
                        */
                        mUART_INTR_RX_MASK_REG &= ~mUART_INTR_RX_NOT_EMPTY;
                        break;
                    }
                    #else
                    {
                        /* Overflow: through away received data element */
                        (void) mUART_RX_FIFO_RD_REG;
                        mUART_rxBufferOverflow = (uint8) mUART_INTR_RX_OVERFLOW;
                    }
                    #endif
                }
                else
                {
                    /* Store received data */
                    mUART_PutWordInRxBuffer(locHead, mUART_RX_FIFO_RD_REG);

                    /* Move head index */
                    mUART_rxBufferHead = locHead;
                }
            }
            while(0u != mUART_GET_RX_FIFO_ENTRIES);

            mUART_ClearRxInterruptSource(mUART_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if (mUART_CHECK_TX_SW_BUFFER)
    {
        if (mUART_CHECK_INTR_TX_MASKED(mUART_INTR_TX_NOT_FULL))
        {
            do
            {
                /* Check for room in TX software buffer */
                if (mUART_txBufferHead != mUART_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (mUART_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if (mUART_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    mUART_TX_FIFO_WR_REG = mUART_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    mUART_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    mUART_DISABLE_INTR_TX(mUART_INTR_TX_NOT_FULL);
                    break;
                }
            }
            while (mUART_SPI_UART_FIFO_SIZE != mUART_GET_TX_FIFO_ENTRIES);

            mUART_ClearTxInterruptSource(mUART_INTR_TX_NOT_FULL);
        }
    }
    #endif

#ifdef mUART_SPI_UART_ISR_EXIT_CALLBACK
    mUART_SPI_UART_ISR_ExitCallback();
#endif /* mUART_SPI_UART_ISR_EXIT_CALLBACK */

}

#endif /* (mUART_SCB_IRQ_INTERNAL) */


/* [] END OF FILE */
