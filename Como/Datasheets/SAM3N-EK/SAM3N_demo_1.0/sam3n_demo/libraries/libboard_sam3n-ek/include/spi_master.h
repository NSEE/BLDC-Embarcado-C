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
 * Implementation of SPI PDC driver.
 *
 */

#ifndef _SPI_MASTER_
#define _SPI_MASTER_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

/** Calculates the value of the SCBR field of the Chip Select Register given MCK and SPCK.*/
#define SPID_CSR_SCBR(mck, spck)    SPI_CSR_SCBR((mck) / (spck))

/** Calculates the value of the DLYBS field of the Chip Select Register given delay in ns and MCK.*/
#define SPID_CSR_DLYBS(mck, delay)  SPI_CSR_DLYBS((((delay) * ((mck) / 1000000)) / 1000) + 1)

/** Calculates the value of the DLYBCT field of the Chip Select Register given delay in ns and MCK.*/
#define SPID_CSR_DLYBCT(mck, delay) SPI_CSR_DLYBCT((((delay) / 32 * ((mck) / 1000000)) / 1000) + 1)

#ifdef __cplusplus
 extern "C" {
#endif

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

/** \brief Spi Master status operation.
 *
 * Each operation return one of the following status.
 */
typedef enum _ESpimStatus
{
	/** Current operation is successful */
	SPIM_OK = 0,
	/** SPI Hw is busy with the previous operation */
	SPIM_BUSY,
	/** No data has been received (No transfer has been initiate by the host) */
	SPIM_EMPTY,
    /** An overrun error has been detected */
    SPIM_ERROR_OVERRUN,
	/** Current operation failed */
	SPIM_ERROR
} ESpimStatus;


extern void SPIM_Handler( void );
extern ESpimStatus SPIM_Initialize( Spi* pSpi );
extern ESpimStatus SPIM_LockCS(Spi *pSpi, uint8_t ucCS);
extern void SPIM_ReleaseCS(Spi *pSpi);
extern void SPIM_ConfigureCS( Spi* pSpi, uint8_t ucCS, uint32_t dwCSR );
extern ESpimStatus SPIM_TransferData(Spi *pSpi, uint16_t wOutData, uint16_t *pwInData);
extern ESpimStatus SPIM_TransferBuffer(Spi *pSpi, const uint16_t *pOutBuffer, uint16_t *pInBuffer, uint16_t wBufferSize);

/* Following functions have to be implemented in the application
 * according to the execution context (RTOS/Superloop...)
 */
extern ESpimStatus SPIM_InitializeContext( Spi* pSpi, IRQn_Type spiId );
extern ESpimStatus SPIM_CreateEotEvent( Spi* pSpi );
extern ESpimStatus SPIM_WaitEotEvent( Spi* pSpi );
extern void SPIM_SignalEotEvent( Spi* pSpi );
extern ESpimStatus SPIM_LockSemaphore( Spi* pSpi );
extern void SPIM_ReleaseSemaphore( Spi* pSpi );
#ifdef __cplusplus
}
#endif

#endif /* #ifndef _SPI_MASTER_ */

