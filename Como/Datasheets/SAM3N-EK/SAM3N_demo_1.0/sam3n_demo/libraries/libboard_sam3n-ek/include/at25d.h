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
 * Interface for the AT25 Serialflash driver.
 *
 */

#ifndef AT25D_H
#define AT25D_H

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/


#ifdef __cplusplus
 extern "C" {
#endif

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

#define AT25_Size(pAt25)            ((pAt25)->pDesc->size)
#define AT25_PageSize(pAt25)        ((pAt25)->pDesc->pageSize)
#define AT25_BlockSize(pAt25)       ((pAt25)->pDesc->blockSize)
#define AT25_Name(pAt25)            ((pAt25)->pDesc->name)
#define AT25_PageNumber(pAt25)      (AT25_Size(pAt25) / AT25_PageSize(pAt25))
#define AT25_BlockNumber(pAt25)     (AT25_Size(pAt25) / AT25_BlockSize(pAt25))
#define AT25_PagePerBlock(pAt25)    (AT25_BlockSize(pAt25) / AT25_PageSize(pAt25))
#define AT25_BlockEraseCmd(pAt25)   ((pAt25)->pDesc->blockEraseCmd)

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

/** \brief Spi Master status operation.
 *
 * Each operation return one of the following status.
 */
typedef enum _EAt25Status
{
    AT25_SUCCESS,    /**< Current operation successful */
    AT25_ERROR_INIT, /**< Initialization error: pAt25->pdesc is not initialized */
    AT25_ERROR_OUT_OF_BOUND, /**< Out of bound operation */
    AT25_ERROR_WRITE,     /**< Write error returned by the serial flash */
    AT25_ERROR_BUSY,      /**< Current operation failed, serial flash is busy */
    AT25_ERROR_PROTECTED, /**< Current operation failed, serial flash is protected */
    AT25_ERROR_SPI, /**< SPI transfer failed */
    AT25_ERROR      /**< Current operation failed */
} EAt25Status;

/** Describes a serial flash device parameters. */
typedef struct _At25Desc {

    /** Device string name. */
    const char *name;
    /** JEDEC ID of device. */
    unsigned int jedecId;
    /** Size of device in bytes. */
    unsigned int size;
    /** Size of one page in bytes. */
    unsigned int pageSize;
    /** Block erase size in bytes. */
    unsigned int blockSize;
    /** Block erase command. */
    unsigned int blockEraseCmd;

} At25Desc;

/** \brief Serial flash Transfer Request prepared by the at25 driver.
 *
 * This structure is sent to the AT25_SendCommand function which is application 
 * dependent. This function transforms At25Cmd request into SpiCmd request.
 */
typedef struct _At25Cmd
{
    /** Data buffer to be sent or received */
    uint32_t *pData;
    /** Serial flash internal address. */
    uint32_t address;
    /** Number of bytes to send/receive. */
    uint16_t dataSize;
    /** Command byte opcode */
    uint8_t cmd;
    /** Size of command (command byte + address bytes + dummy bytes) in bytes. */
    uint8_t cmdSize;
} At25Cmd;    

/**
 * Serial flash driver structure. Holds the current state of the driver,
 * including the current command and the descriptor for the underlying device.
 */
typedef struct _At25 {

    /** Pointer to the underlying SPI Hardware peripheral. */
    Spi *pSpi;
    /** Pointer to a descriptor for the serial firmware flash device. */
    const At25Desc *pDesc;
    /** Active chip select for the connect flash device */
    uint8_t cs;
} At25;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
extern EAt25Status AT25_Initialize(At25 *pAt25, Spi *, uint8_t cs);

extern EAt25Status AT25_WaitReady(At25 *pAt25);

extern EAt25Status AT25_ReadStatus(At25 *pAt25, uint8_t *pAt25Status);

extern EAt25Status AT25_WriteStatus(At25 *pAt25, uint8_t at25Status);

extern EAt25Status AT25_ReadJedecId(At25 *pAt25, uint32_t *pJedecId);

extern EAt25Status AT25_EnableWrite(At25 *pAt25);

extern EAt25Status AT25_Unprotect(At25 *pAt25);

extern EAt25Status AT25_EraseChip(At25 *pAt25);

extern EAt25Status AT25_EraseBlock(At25 *pAt25, uint32_t address);

extern EAt25Status AT25_Write(
    At25 *pAt25,
    uint32_t *pData,
    uint16_t size,
    uint32_t address);

extern EAt25Status AT25_Read(
    At25 *pAt25,
    uint32_t *pData,
    uint16_t size,
    uint32_t address);

extern uint32_t ReadJedecId(At25 *pAt25);

extern EAt25Status AT25_GetDeviceDescriptor(At25 *pAt25);

/*----------------------------------------------------------------------------
 * These functions have to be defined by the application. Their implementation
 * depends on the application context 
 *----------------------------------------------------------------------------*/

extern void AT25_Configure(At25 *pAt25, Spi *pSpi, unsigned char cs);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef AT25D_H */

