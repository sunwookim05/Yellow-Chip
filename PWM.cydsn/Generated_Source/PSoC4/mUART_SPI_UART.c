/***************************************************************************//**
* \file mUART_SPI_UART.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mUART_PVT.h"
#include "mUART_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(mUART_INTERNAL_RX_SW_BUFFER_CONST)
    /* Start index to put data into the software receive buffer.*/
    volatile uint32 mUART_rxBufferHead;
    /* Start index to get data from the software receive buffer.*/
    volatile uint32 mUART_rxBufferTail;
    /**
    * \addtogroup group_globals
    * \{
    */
    /** Sets when internal software receive buffer overflow
    *  was occurred.
    */
    volatile uint8  mUART_rxBufferOverflow;
    /** \} globals */
#endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

#if(mUART_INTERNAL_TX_SW_BUFFER_CONST)
    /* Start index to put data into the software transmit buffer.*/
    volatile uint32 mUART_txBufferHead;
    /* Start index to get data from the software transmit buffer.*/
    volatile uint32 mUART_txBufferTail;
#endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */

#if(mUART_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 mUART_rxBufferInternal[mUART_INTERNAL_RX_BUFFER_SIZE];
#endif /* (mUART_INTERNAL_RX_SW_BUFFER) */

#if(mUART_INTERNAL_TX_SW_BUFFER)
    volatile uint8 mUART_txBufferInternal[mUART_TX_BUFFER_SIZE];
#endif /* (mUART_INTERNAL_TX_SW_BUFFER) */


#if(mUART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: mUART_SpiUartReadRxData
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer.
    *   - RX software buffer is disabled: Returns data element retrieved from
    *     RX FIFO. Undefined data will be returned if the RX FIFO is empty.
    *   - RX software buffer is enabled: Returns data element from the software
    *     receive buffer. Zero value is returned if the software receive buffer
    *     is empty.
    *
    * \return
    *  Next data element from the receive buffer. 
    *  The amount of data bits to be received depends on RX data bits selection 
    *  (the data bit counting starts from LSB of return value).
    *
    * \globalvars
    *  mUART_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  mUART_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 mUART_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

    #if (mUART_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (mUART_CHECK_RX_SW_BUFFER)
        {
            if (mUART_rxBufferHead != mUART_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (mUART_rxBufferTail + 1u);

                if (mUART_INTERNAL_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = mUART_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                mUART_rxBufferTail = locTail;

                #if (mUART_CHECK_UART_RTS_CONTROL_FLOW)
                {
                    /* Check if RX Not Empty is disabled in the interrupt */
                    if (0u == (mUART_INTR_RX_MASK_REG & mUART_INTR_RX_NOT_EMPTY))
                    {
                        /* Enable RX Not Empty interrupt source to continue
                        * receiving data into software buffer.
                        */
                        mUART_INTR_RX_MASK_REG |= mUART_INTR_RX_NOT_EMPTY;
                    }
                }
                #endif

            }
        }
        #else
        {
            /* Read data from RX FIFO */
            rxData = mUART_RX_FIFO_RD_REG;
        }
        #endif

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: mUART_SpiUartGetRxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in the receive buffer. This does not include the hardware RX FIFO.
    *
    * \return
    *  Number of received data elements.
    *
    * \globalvars
    *  mUART_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  mUART_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 mUART_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
    #if (mUART_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (mUART_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (mUART_CHECK_RX_SW_BUFFER)
        {
            locHead = mUART_rxBufferHead;

            if(locHead >= mUART_rxBufferTail)
            {
                size = (locHead - mUART_rxBufferTail);
            }
            else
            {
                size = (locHead + (mUART_INTERNAL_RX_BUFFER_SIZE - mUART_rxBufferTail));
            }
        }
        #else
        {
            size = mUART_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: mUART_SpiUartClearRxBuffer
    ****************************************************************************//**
    *
    *  Clears the receive buffer and RX FIFO.
    *
    * \globalvars
    *  mUART_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  mUART_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    void mUART_SpiUartClearRxBuffer(void)
    {
        #if (mUART_CHECK_RX_SW_BUFFER)
        {
            /* Lock from component interruption */
            mUART_DisableInt();

            /* Flush RX software buffer */
            mUART_rxBufferHead = mUART_rxBufferTail;
            mUART_rxBufferOverflow = 0u;

            mUART_CLEAR_RX_FIFO;
            mUART_ClearRxInterruptSource(mUART_INTR_RX_ALL);

            #if (mUART_CHECK_UART_RTS_CONTROL_FLOW)
            {
                /* Enable RX Not Empty interrupt source to continue receiving
                * data into software buffer.
                */
                mUART_INTR_RX_MASK_REG |= mUART_INTR_RX_NOT_EMPTY;
            }
            #endif
            
            /* Release lock */
            mUART_EnableInt();
        }
        #else
        {
            mUART_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (mUART_RX_DIRECTION) */


#if(mUART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: mUART_SpiUartWriteTxData
    ****************************************************************************//**
    *
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *   The amount of data bits to be transmitted depends on TX data bits selection 
    *   (the data bit counting starts from LSB of txDataByte).
    *
    * \globalvars
    *  mUART_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  mUART_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void mUART_SpiUartWriteTxData(uint32 txData)
    {
    #if (mUART_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (mUART_CHECK_TX_SW_BUFFER)
        {
            /* Put data directly into the TX FIFO */
            if ((mUART_txBufferHead == mUART_txBufferTail) &&
                (mUART_SPI_UART_FIFO_SIZE != mUART_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                mUART_TX_FIFO_WR_REG = txData;
            }
            /* Put data into TX software buffer */
            else
            {
                /* Head index to put data */
                locHead = (mUART_txBufferHead + 1u);

                /* Adjust TX software buffer index */
                if (mUART_TX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                /* Wait for space in TX software buffer */
                while (locHead == mUART_txBufferTail)
                {
                }

                /* TX software buffer has at least one room */

                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                mUART_ClearTxInterruptSource(mUART_INTR_TX_NOT_FULL);

                mUART_PutWordInTxBuffer(locHead, txData);

                mUART_txBufferHead = locHead;

                /* Check if TX Not Full is disabled in interrupt */
                if (0u == (mUART_INTR_TX_MASK_REG & mUART_INTR_TX_NOT_FULL))
                {
                    /* Enable TX Not Full interrupt source to transmit from software buffer */
                    mUART_INTR_TX_MASK_REG |= (uint32) mUART_INTR_TX_NOT_FULL;
                }
            }
        }
        #else
        {
            /* Wait until TX FIFO has space to put data element */
            while (mUART_SPI_UART_FIFO_SIZE == mUART_GET_TX_FIFO_ENTRIES)
            {
            }

            mUART_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: mUART_SpiUartPutArray
    ****************************************************************************//**
    *
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * \param wrBuf: pointer to an array of data to be placed in transmit buffer. 
    *  The width of the data to be transmitted depends on TX data width selection 
    *  (the data bit counting starts from LSB for each array element).
    * \param count: number of data elements to be placed in the transmit buffer.
    *
    * \globalvars
    *  mUART_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  mUART_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void mUART_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for (i=0u; i < count; i++)
        {
            mUART_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: mUART_SpiUartGetTxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of elements currently in the transmit buffer.
    *   - TX software buffer is disabled: returns the number of used entries in
    *     TX FIFO.
    *   - TX software buffer is enabled: returns the number of elements currently
    *     used in the transmit buffer. This number does not include used entries in
    *     the TX FIFO. The transmit buffer size is zero until the TX FIFO is
    *     not full.
    *
    * \return
    *  Number of data elements ready to transmit.
    *
    * \globalvars
    *  mUART_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  mUART_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    uint32 mUART_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
    #if (mUART_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (mUART_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (mUART_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = mUART_txBufferTail;

            if (mUART_txBufferHead >= locTail)
            {
                size = (mUART_txBufferHead - locTail);
            }
            else
            {
                size = (mUART_txBufferHead + (mUART_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = mUART_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: mUART_SpiUartClearTxBuffer
    ****************************************************************************//**
    *
    *  Clears the transmit buffer and TX FIFO.
    *
    * \globalvars
    *  mUART_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  mUART_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void mUART_SpiUartClearTxBuffer(void)
    {
        #if (mUART_CHECK_TX_SW_BUFFER)
        {
            /* Lock from component interruption */
            mUART_DisableInt();

            /* Flush TX software buffer */
            mUART_txBufferHead = mUART_txBufferTail;

            mUART_INTR_TX_MASK_REG &= (uint32) ~mUART_INTR_TX_NOT_FULL;
            mUART_CLEAR_TX_FIFO;
            mUART_ClearTxInterruptSource(mUART_INTR_TX_ALL);

            /* Release lock */
            mUART_EnableInt();
        }
        #else
        {
            mUART_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (mUART_TX_DIRECTION) */


/*******************************************************************************
* Function Name: mUART_SpiUartDisableIntRx
****************************************************************************//**
*
*  Disables the RX interrupt sources.
*
*  \return
*   Returns the RX interrupt sources enabled before the function call.
*
*******************************************************************************/
uint32 mUART_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = mUART_GetRxInterruptMode();

    mUART_SetRxInterruptMode(mUART_NO_INTR_SOURCES);

    return (intSource);
}


/*******************************************************************************
* Function Name: mUART_SpiUartDisableIntTx
****************************************************************************//**
*
*  Disables TX interrupt sources.
*
*  \return
*   Returns TX interrupt sources enabled before function call.
*
*******************************************************************************/
uint32 mUART_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = mUART_GetTxInterruptMode();

    mUART_SetTxInterruptMode(mUART_NO_INTR_SOURCES);

    return (intSourceMask);
}


#if(mUART_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: mUART_PutWordInRxBuffer
    ****************************************************************************//**
    *
    *  Stores a byte/word into the RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param index:      index to store data byte/word in the RX buffer.
    *  \param rxDataByte: byte/word to store.
    *
    *******************************************************************************/
    void mUART_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if (mUART_ONE_BYTE_WIDTH == mUART_rxDataBits)
        {
            mUART_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            mUART_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            mUART_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: mUART_GetWordFromRxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \return
    *   Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 mUART_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if (mUART_ONE_BYTE_WIDTH == mUART_rxDataBits)
        {
            value = mUART_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) mUART_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)mUART_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }


    /*******************************************************************************
    * Function Name: mUART_PutWordInTxBuffer
    ****************************************************************************//**
    *
    *  Stores byte/word into the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx:        index to store data byte/word in the TX buffer.
    *  \param txDataByte: byte/word to store.
    *
    *******************************************************************************/
    void mUART_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if (mUART_ONE_BYTE_WIDTH == mUART_txDataBits)
        {
            mUART_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            mUART_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            mUART_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: mUART_GetWordFromTxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx: index to get data byte/word from the TX buffer.
    *
    *  \return
    *   Returns byte/word read from the TX buffer.
    *
    *******************************************************************************/
    uint32 mUART_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if (mUART_ONE_BYTE_WIDTH == mUART_txDataBits)
        {
            value = (uint32) mUART_txBuffer[idx];
        }
        else
        {
            value  = (uint32) mUART_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) mUART_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }

#endif /* (mUART_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
