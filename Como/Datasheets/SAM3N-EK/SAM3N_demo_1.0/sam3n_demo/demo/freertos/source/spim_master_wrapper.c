/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \file
 *
 * Implementation of SPI master wrapper layer.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include "libFreeRTOS.h"

/*----------------------------------------------------------------------------
 *        Internal structures
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/** Semaphore to protect the SPI periph */
static xSemaphoreHandle spiSemaphore;
/** Semaphore used as a semaphore event */
static xSemaphoreHandle eotEvent;

extern ESpimStatus SPIM_InitializeContext( Spi* pSpi, IRQn_Type spiId )
{
    vSemaphoreCreateBinary( spiSemaphore );
    if( spiSemaphore == NULL )
        return SPIM_ERROR;

    vSemaphoreCreateBinary( eotEvent );
    if( eotEvent == NULL )
        return SPIM_ERROR;

    /* Configure the interrupt with the according priority */
    NVIC_SetPriority( spiId, configMAX_SYSCALL_INTERRUPT_PRIORITY + 16 ) ;

    return SPIM_OK;
}

extern ESpimStatus SPIM_LockSemaphore( Spi* pSpi )
{
    /* This is the place to test a semaphore as upper layer tasks can do
     * concurrent access to the SPI hw */
    if( xSemaphoreTake( spiSemaphore, portMAX_DELAY ) != pdTRUE )
        return SPIM_BUSY;

    return SPIM_OK;
}

extern void SPIM_ReleaseSemaphore( Spi* pSpi )
{
    xSemaphoreGive( spiSemaphore );
}


extern ESpimStatus SPIM_CreateEotEvent( Spi* pSpi )
{
    /* Use a semaphore as an event  */
    if( xSemaphoreTake( eotEvent, portMAX_DELAY ) != pdTRUE )
        return SPIM_ERROR;
    return SPIM_OK;
}

extern void SPIM_SignalEotEvent( Spi* pSpi )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Release the semaphore to notify the end of transfer */
    xSemaphoreGiveFromISR( eotEvent, &xHigherPriorityTaskWoken );

    /* We may want to switch to the SPIM task, if this message has made
    it the highest priority task that is ready to execute. */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

extern ESpimStatus SPIM_WaitEotEvent( Spi* pSpi )
{
    /* Poll the end of transfer waiting that ISR releases the semaphore  */
     if( xSemaphoreTake( eotEvent, portMAX_DELAY ) != pdPASS )
         return SPIM_ERROR;
     xSemaphoreGive( eotEvent );
     return SPIM_OK;
}
