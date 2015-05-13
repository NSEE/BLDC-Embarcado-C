/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation

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
 * Interface for SDMMC SPI driver.
 */

#ifndef SDSPID_H
#define SDSPID_H
/** \addtogroup sdmmc_spi_hal
 *@{
 */

/*------------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>

/*------------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/** \addtogroup sdspid_rc SDMMC SPI return code
 *      @{
 */
/** Command process OK */
#define SDSPI_SUCCESS       0
/** Command process error */
#define SDSPI_ERROR         1
/** Card has no response */
#define SDSPI_NO_RESPONSE   2
/** No response, time out */
#define SDSPI_ERROR_TIMEOUT SDSPI_NO_RESPONSE
/** Card is still busy */
#define SDSPI_BUSY          3
/** CRC error for data */
#define SDSPI_ERROR_CRC     4
/**     @}*/

/** \addtogroup sdspid_spi_cfg SDMMC SPI Configurations
 *      @{
 */
/** CSR register value for SPI configuration */
#define SDSPI_CSR(mck, spck) \
    (SPI_CSR_NCPHA | SPID_CSR_DLYBCT(mck, 20) \
     | SPID_CSR_DLYBS(mck, 20) | SPID_CSR_SCBR(mck, spck) \
     | 0 /*AT91C_SPI_CSAAT*/ \
    )
/**     @}*/

/** \addtogroup sdmmc_cmd_op SDMMC Command Operation Settings
 *      @{
 */
/** Execute power on initialization */
#define SDMMC_CMD_bmPOWERON     (0x1     )
/** Send command & get response */
#define SDMMC_CMD_bmCOMMAND     (0x1 << 1)
/** Data transfer option mask */
#define SDMMC_CMD_bmDATAMASK    (0x3 << 2)
/** No data transfer */
#define SDMMC_CMD_bmNODATA      (0x0 << 2)
/** Read data from card */
#define SDMMC_CMD_RX             0x1
/** Bits in cmd op that indicate read */
#define SDMMC_CMD_bmDATARX      (0x1 << 2)
/** Write data to card */
#define SDMMC_CMD_TX             0x2
/** Bits in cmd op that indicate write */
#define SDMMC_CMD_bmDATATX      (0x2 << 2)
/** Stop data transfer */
#define SDMMC_CMD_STOP
/** Bits in cmd op that indicate stop transfer */
#define SDMMC_CMD_bmSTOPXFR     (0x2 << 2)
/** Response type mask for cmd op */
#define SDMMC_CMD_bmRESPMASK    (0x7 << 4)
/** CRC is used for SDMMC SPI commands */
#define SDMMC_CMD_bmCRC         (0x1 << 7)

/** Do power on initialize */
#define SDMMC_CMD_POWERONINIT   (SDMMC_CMD_bmPOWERON)
/** Data transfer only, read */
#define SDMMC_CMD_DATARX        (SDMMC_CMD_bmDATARX)
/** Data transfer only, write */
#define SDMMC_CMD_DATATX        (SDMMC_CMD_bmDATATX)
/** Send Command without data */
#define SDMMC_CMD_CNODATA(R)    ( SDMMC_CMD_bmCOMMAND \
                                |(((R)&0x7)<<4))
/** Send Command with data read */
#define SDMMC_CMD_CDATARX(R)    ( SDMMC_CMD_bmCOMMAND \
                                | SDMMC_CMD_bmDATARX \
                                | (((R)&0x7)<<4))
/** Send Command with data write */
#define SDMMC_CMD_CDATATX(R)    ( SDMMC_CMD_bmCOMMAND \
                                | SDMMC_CMD_bmDATATX \
                                | (((R)&0x7)<<4))
/** Send Stop token for SDMMC SPI */
#define SDMMC_CMD_STOPTOKEN     (SDMMC_CMD_bmSTOPXFR)
/**     @}*/

/*------------------------------------------------------------------------------
 *         Types
 *----------------------------------------------------------------------------*/

/** SDMMC end-of-transfer callback function. */
typedef void (*SdmmcCallback)(uint8_t status, void *pCommand);

/**
 * Sdmmc command operation settings.
 */
typedef union _SdmmcCmdOperation {
    uint8_t bVal;
    struct {
        uint8_t powerON:1, /**< Do power on initialize */
                sendCmd:1, /**< Send SD/MMC command */
                xfrData:2, /**< Send/Stop data transfer */
                respType:3,/**< Response type */
                crcON:1;   /**< CRC is used (SPI only) */
    } bmBits;
} SdmmcCmdOp;

/**
 * Sdmmc command.
 */
typedef struct _SdmmcCommand {
    /** Optional user-provided callback function. */
    SdmmcCallback callback;
    /** Optional argument to the callback function. */
    void *pArg;
    /** Data buffer, with MCI_DMA_ENABLE defined 1, the buffer can be
     * 1, 2 or 4 bytes aligned. It has to be 4 byte aligned if no DMA.
     */
    uint8_t *pData;
    /** Size of data block in bytes. */
    uint16_t blockSize;
    /** Number of blocks to be transfered */
    uint16_t nbBlock;
    /** Response buffer. */
    uint32_t  *pResp;
    /** Command argument. */
    uint32_t   arg;
    /**< Command index */
    uint8_t    cmd;
    /**< Command operation settings */
    SdmmcCmdOp cmdOp;
    /**< Command return status */
    uint8_t    status;
    /**< Command state */
    volatile uint8_t state;
} SdmmcCommand, SdSpiCmd;

/**
 * SPI driver structure. Holds the internal state of the SPI driver and
 * prevents parallel access to a SPI peripheral.
 */
typedef struct _SdSpid {

    /** Pointer to a SPI driver. */
    void *pSpid;
    /** Pointer to processing command */
    SdSpiCmd *pCmd;
    /** additional buffer */
    uint8_t buffer[3];
    /** Active chip select for the device */
    uint8_t bCs;

} SdSpid;
typedef SdSpid SdSpi;

extern uint32_t SDSPI_SendCommand(SdSpid *pSdSpi, SdmmcCommand *pCmd);
extern uint32_t SDSPI_AsynCommand(SdSpid *pSdSpi, SdmmcCommand *pCmd);
extern uint32_t SDSPI_IsBusy(SdSpid *pSdSpi);

extern void SDSPI_Configure(
    SdSpid *pSdSpi, void *pSpid, uint8_t bCs,
    uint32_t mck, uint32_t pck);

extern uint32_t SDSPI_SetClock(
    SdSpid * pSdSpid, uint32_t dwMck, uint32_t dwPck);

/**@}*/
#endif

