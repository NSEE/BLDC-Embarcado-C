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

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <string.h>

/*----------------------------------------------------------------------------
 *         External support prototypes
 *----------------------------------------------------------------------------*/

/* ---------------- CRC ---------------- */
//#define SDSPI_CRC_SUPPORT
#ifdef SDSPI_CRC_SUPPORT
#include <sdmmc_crc.h>
#define crc7_calc                   crc7_calc
#define crc16_calc                  crc16_ccitt_calc
#endif

/*----------------------------------------------------------------------------
 *         Macros
 *----------------------------------------------------------------------------*/

/* Data Response Token RC */
#define SDSPI_DATA_NO_RESP  0x1
#define SDSPI_DATA_ACCEPTED 0x5
#define SDSPI_DATA_CRC_ERR  0xB
#define SDSPI_DATA_WR_ERR   0xD

/* Data Tokens */
#define SDSPI_START_BLOCK_1 0xFE  /**< Single/Multiple read, single write */
#define SDSPI_START_BLOCK_2 0xFC  /**< Multiple block write */
#define SDSPI_STOP_TRAN     0xFD  /**< Stop token */

/**
 * Convert SD MCI command to a SPI mode command token.
 * \param pCmdToken Pointer to the SD command token buffer.
 * \param pCmd      Pointer to the SD command instance.
 */
static void _MakeSdSpiCmd(uint8_t *pCmdToken,
                          SdmmcCommand *pCmd)
{
    uint8_t sdCmdNum = pCmd->cmd & 0x3f;
  #ifdef SDSPI_CRC_SUPPORT
    uint8_t crc = 0;
  #endif

    *pCmdToken = sdCmdNum | 0x40;
    *(pCmdToken+1) = (pCmd->arg >> 24) & 0xff;
    *(pCmdToken+2) = (pCmd->arg >> 16) & 0xff;
    *(pCmdToken+3) = (pCmd->arg >> 8)  & 0xff;
    *(pCmdToken+4) =  pCmd->arg        & 0xff;

  #ifdef SDSPI_CRC_SUPPORT
    if (pCmd->cmdOp.bmBits.crcON
        || pCmd->cmd == 0
        || pCmd->cmd == 8) {
        crc = crc7_calc(pCmdToken, 5);
    }
    *(pCmdToken+5) = (crc << 1) | 1;
  #else
    /* Valid reset command CRC = 0x95 */
    *(pCmdToken+5) = (pCmd->cmd == 0) ? 0x95 :
                   ( (pCmd->cmd == 8) ? 0x87 : 0 );
  #endif


    TRACE_DEBUG_WP("C %x %x %x %x %x %x\n\r",
        pCmdToken[0], pCmdToken[1], pCmdToken[2],
        pCmdToken[3], pCmdToken[4], pCmdToken[5]);
}

/**
 * Write & Read data
 * \param pSdSpi  Pointer to a SD SPI driver instance.
 * \param pData  Data pointer.
 * \param size Data size.
 */
static uint32_t _SyncBytes(SdSpid *pSdSpid, uint8_t *pData, uint16_t size)
{
    Spi *pSpid = (Spi*)pSdSpid->pSpid;
    uint32_t rc;
#if 0
  { uint32_t i = 0;
    for(i = 0; i < size; i ++) {
      uint16_t tmp;
      do {
        rc = SPIM_TransferData(pSpid, pData[i], &tmp);
      } while(rc == SPIM_BUSY);
      pData[i] = tmp;
      if (rc != SPIM_OK) break;
    }
  }
#else
  {
    do {
      rc = SPIM_TransferBuffer(pSpid, (const uint16_t *)pData, (uint16_t *)pData, size);
    } while (rc == SPIM_BUSY);
    if (rc != SPIM_OK) return SDSPI_ERROR;
  }
#endif

    return rc;
}

/**
 * Read data on SPI data bus;
 * \param pSdSpi  Pointer to a SD SPI driver instance.
 * \param pData  Data pointer.
 * \param size Data size.
 */
static uint32_t _SyncRead(SdSpid *pSdSpid, uint8_t *pData, uint32_t size)
{
    /* MOSI should hold high during read,
       or there will be wrong data in received data. */
    memset(pData, 0xff, size);
    return _SyncBytes(pSdSpid, pData, size);
}

/**
 * Wait the card to be ready
 * \param  pSdSpi Pointer to SdSpid instance.
 * \return SDSPI_SUCCESS or error code.
 */
static uint8_t _WaitBusy(SdSpid *pSdSpi)
{
    uint32_t to = 50000000/8/(1000/200); /* 200ms for 50 MHz clock */
    uint8_t  rc = SDSPI_BUSY;
    uint8_t  d;

    while (to --) {
        rc = _SyncRead(pSdSpi, &d, 1);
        if (rc) break;
        if (d == 0xFF) {
            return SDSPI_SUCCESS;
        }
    }
    return rc;
}

/**
 * Send SD command and get its response.
 * \param  pSdSpi Pointer to SdSpid instance.
 * \return SDSPI_SUCCESS or error code.
 */
static uint8_t _SendCmd(SdSpid *pSdSpi)
{
    SdmmcCommand *pCmd = pSdSpi->pCmd;
    uint32_t to = 8; /* NCR timeout max 8 */
    uint8_t rc;
    uint8_t CmdToken[6];
    uint8_t CmdResp[8];
    uint8_t more = 0;

    TRACE_DEBUG("Cmd%d\n\r", pCmd->cmd & 0x3F);

    _MakeSdSpiCmd(CmdToken, pCmd);

    /* Send command */
    rc = _SyncBytes(pSdSpi, CmdToken, 6);
    if (rc) {
        TRACE_INFO("ErrSendC\n\r");
        return rc;
    }

    /* Get response */
    memset(CmdResp, 0, 8);
    /* Wait for response start bit */
    do {
        rc = _SyncRead(pSdSpi, &CmdResp[0], 1);
        if (rc) {
            TRACE_INFO("ErrGetCR.S\n\r");
            return rc;
        }
        if ((CmdResp[0]&0x80) == 0) break;
    } while(-- to);
    /* Not timeout, get response */
    if (to) {
        switch(pCmd->cmdOp.bmBits.respType) {
            case 1:           break;
            case 2: more = 1; break;
            case 3:
            case 7: more = 4; break;
            default:
                TRACE_ERROR("rspType %x\n\r", pCmd->cmdOp.bmBits.respType);
                return SDSPI_ERROR;
        }
        /* Read more data */
        if (more) {
            rc = _SyncRead(pSdSpi, &CmdResp[1], more);
            if (rc) {
                TRACE_INFO("ErrGetCR.D");
                return rc;
            }
        }
        /* Parse response */
        if (pCmd->pResp) {
            pCmd->pResp[0] = 0;
            switch (more) {
                case 4:
                    pCmd->pResp[1]  = CmdResp[4];
                    pCmd->pResp[0] |= (CmdResp[2] << 16)
                                    | (CmdResp[3] << 24);
                case 1:
                    pCmd->pResp[0] |= CmdResp[1] << 8;
                case 0:
                    pCmd->pResp[0] |= CmdResp[0];
                    break;
            }
        }
        return SDSPI_SUCCESS;
    }
    else
        return SDSPI_ERROR_TIMEOUT;

}

/**
 * Read data
 * \param  pSdSpi Pointer to SdSpid instance.
 * \param  pCmd   Pointer to SdmmcCommand instance.
 * \return SDSPI_SUCCESS or error code.
 */
static uint8_t _ReadBlocks(SdSpid *pSdSpi)
{
    SdmmcCommand *pCmd = pSdSpi->pCmd;
    uint32_t to = 50000000/8/(1000/100); /* 100ms for 50 MHz clock */
    uint8_t  header, crc[2];
    uint8_t  rc;
    uint32_t i;
    uint8_t  *pData = pCmd->pData;
    TRACE_DEBUG("SdSpiRD\n\r");

    if (pCmd->pData == 0) {
        TRACE_ERROR("SdSpiRD buffer not allocated\n\r");
        return SDSPI_ERROR;
    }

    for (i = 0; i < pCmd->nbBlock; i ++) {

        /* Wait data header */
        do {
            rc = _SyncRead(pSdSpi, &header, 1);
            if (rc) {
                TRACE_INFO("spiRD.Hdr\n\r");
                return rc;
            }
            TRACE_DEBUG("Hdr %x\n\r", header);
            /* Data start */
            if (header == SDSPI_START_BLOCK_1) break;
            /* Error token */
            else if ((header & 0xf0) == 0) {
                TRACE_ERROR("spiRD.ErrHdr %x @ %u\n\r", (unsigned int)header, (unsigned int)to);
                return SDSPI_ERROR;
            }
        } while (-- to);
        /* Time out */
        if (to == 0) {
            TRACE_ERROR("spiRD.TO\n\r");
            return SDSPI_ERROR_TIMEOUT;
        }
      #if 0
      { Spim *pSpim = pSdSpi->pSpim;
        SpimCmd spimCmd;
        memset(pData, 0xff, pCmd->blockSize);
        memset(crc,   0xff, 2);
        spimCmd.pTxBuffer = spimCmd.pRxBuffer = pData;
        spimCmd.pNextTxBuffer = spimCmd.pNextRxBuffer = crc;
        spimCmd.wBufferSize = pCmd->blockSize;
        spimCmd.wNextBufferSize = 2;
        do {
          rc = SPIMW_SendBuffer(pSpim, &spimCmd);
        } while (rc == SPIM_BUSY);
        if (rc) {
            TRACE_INFO("spiRD.Data\n\r");
            return rc;
        }
      }
      #else
        /* Read data */
        rc = _SyncRead(pSdSpi, pData, pCmd->blockSize);
        if (rc) {
            TRACE_INFO("spiRD.Data\n\r");
            return rc;
        }

        /* Read CRC */
        rc = _SyncRead(pSdSpi, crc, 2);
        if (rc) {
            TRACE_INFO("spiRD.crc %x\n\r", rc);
            return rc;
        }
      #endif

      #ifdef SDSPI_CRC_SUPPORT
        /* Check CRC */
        if (pCmd->cmdOp.bmBits.crcON) {
            uint16_t wCRC = crc16_calc(pData, pCmd->blockSize);
            if (   crc[0] != ((wCRC >> 8) & 0xff)
                || crc[1] !=   wCRC & 0xff) {
                TRACE_ERROR("spiRD.crcErr");
                return SDSPI_ERROR_CRC;
            }
        }
      #else
        /* No CRC operation */
      #endif

        /* Next block */
        pData += pCmd->blockSize;
    }

    return SDSPI_SUCCESS;
}

/**
 * Write data
 * \param  pSdSpi Pointer to SdSpid instance.
 * \param  pCmd   Pointer to SdmmcCommand instance.
 * \return SDSPI_SUCCESS or error code.
 */
static uint8_t _WriteBlocks(SdSpid *pSdSpi)
{
    SdmmcCommand *pCmd = pSdSpi->pCmd;
    uint32_t to = 8; /* NCR timeout max 8 */
    uint8_t  header, crc[2] = {0, 0};
    uint8_t  rc;
    uint32_t i;
    uint8_t  *pData = pCmd->pData;
    TRACE_DEBUG("SdSpiWR\n\r");

    for (i = 0; i < pCmd->nbBlock; i ++) {

        /* Check if card is ready */
        rc = _WaitBusy(pSdSpi);
        if (rc) {
            TRACE_INFO("sdWR.busy\n\r");
            return rc;
        }

        /* Prepare header & crc */
        if (  ((pCmd->cmdOp.bmBits.sendCmd == 0)
                && (pCmd->cmdOp.bmBits.xfrData == 1 || pCmd->cmdOp.bmBits.xfrData == 2))
            ||(pCmd->cmd == 25))
            header = SDSPI_START_BLOCK_2;
        else
            header = SDSPI_START_BLOCK_1;

        rc = _SyncBytes(pSdSpi, &header, 1);
        if (rc) {
            TRACE_INFO("spiWR.hdr\n\r");
            return rc;
        }

      #ifdef SDSPI_CRC_SUPPORT
        if (pCmd->cmdOp.bmBits.crcON) {
            uint16_t wCRC = crc16_calc(pData, pCmd->blockSize);
            crc[0] = (wCRC >> 8) & 0xff;
            crc[1] =  wCRC       & 0xff;
        }
      #else
        /* No crc calculation */
      #endif

      #if 0
      { Spim *pSpim = pSdSpi->pSpim;
        SpimCmd spimCmd;
        spimCmd.pTxBuffer = spimCmd.pRxBuffer = pData;
        spimCmd.pNextTxBuffer = spimCmd.pNextRxBuffer = crc;
        spimCmd.wBufferSize = pCmd->blockSize;
        spimCmd.wNextBufferSize = 2;
        do {
          rc = SPIMW_SendBuffer(pSpim, &spimCmd);
        } while (rc == SPIM_BUSY);
        if (rc) {
            TRACE_INFO("spiWR.data\n\r");
            return rc;
        }
      }
      #else
        rc = _SyncBytes(pSdSpi, pData, pCmd->blockSize);
        if (rc) {
            TRACE_INFO("spiWR.data\n\r");
            return rc;
        }

        rc = _SyncBytes(pSdSpi, crc, 2);
        if (rc) {
            TRACE_INFO("spiWR.crc\n\r");
            return rc;
        }
      #endif

        /* Wait data response in 8 clocks */
        do {
            rc = _SyncRead(pSdSpi, &header, 1);
            if (rc) {
                TRACE_INFO("spiWR.Rsp %x\n\r", rc);
                return rc;
            }
            if ((header & 0x11) == 0x1) break;
        } while (-- to);
        /* Wait response timeout */
        if (to == 0) {
            TRACE_ERROR("sdWR.DRspTO\n\r");
            return SDSPI_ERROR_TIMEOUT;
        }
        /* Check response status */
        switch(header & 0x1F) {

        case SDSPI_DATA_ACCEPTED:   break;

        case SDSPI_DATA_CRC_ERR:
            TRACE_ERROR("sdWR.DR.CRC\n\r");
            return SDSPI_ERROR;

        case SDSPI_DATA_WR_ERR:
            TRACE_ERROR("sdWR.DR.ERR\n\r");
            return SDSPI_ERROR;

        default:
            TRACE_ERROR("sdWR.DR.RESP %x\n\r", header);
            return SDSPI_ERROR;
        }

        /* Next data block */
        pData += pCmd->blockSize;
    }
    return SDSPI_SUCCESS;
}

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 * Initializes the SD Spi structure and the corresponding SPI hardware.
 * \param pSdSpid   Pointer to a SdSpid instance.
 * \param pSpid     Associated SPI Master driver.
 * \param bCs       Chip Select value.
 * \param dwMck     MCK for clock configuration.
 * \param dwPck     Expected peripheral clock.
 */
void SDSPI_Configure(SdSpid * pSdSpid,
                     void * pSpid,
                     uint8_t bCs,
                     uint32_t dwMck,
                     uint32_t dwPck)
{
    /* Sanity checks */
    assert(pSdSpid);
    assert(pSpid);
    assert(bCs < 4);
    assert(SPID_CSR_SCBR(dwMck, dwPck));

    /* Configure the SPI chip select */
    SPIM_ConfigureCS(pSpid, bCs, SDSPI_CSR(dwMck, dwPck));

    /* Initialize struct */
    pSdSpid->pSpid = pSpid;
    pSdSpid->pCmd  = 0;
    pSdSpid->bCs   = bCs;
}

/**
 * Set clock frequency.
 * \param pSdSpid Pointer to SdSpid instance.
 * \param dwMck   Current MCK used to generate SPI clock.
 * \param dwPck   Expected SPI clock frequency.
 */
uint32_t SDSPI_SetClock(SdSpid * pSdSpid, uint32_t dwMck, uint32_t dwPck)
{
    Spi *pSpid;

    assert(pSdSpid);

    pSpid = pSdSpid->pSpid;
    assert(pSpid);

    /* Check busy ?? */

    if (SPID_CSR_SCBR(dwMck, dwPck) == 0)
        return SDSPI_ERROR;

    SPIM_ConfigureCS(pSpid, pSdSpid->bCs, SDSPI_CSR(dwMck, dwPck));

    return SDSPI_SUCCESS;
}

/**
 * Execute a SPI master transfer. This is a blocking function.
 * Returns 0 if the transfer has been started successfully; otherwise returns
 * error.
 * \param pSdSpid  Pointer to a SdSpid instance.
 * \param pCommand Pointer to the SPI command to execute.
 */
uint32_t SDSPI_SendCommand(SdSpid *pSdSpid, SdmmcCommand *pCommand)
{
    Spi *pSpid;
    uint8_t error = SDSPI_SUCCESS;

    assert(pSdSpid);
    assert(pCommand);

    pSpid = pSdSpid->pSpid;
    assert(pSpid);

    TRACE_DEBUG("SdSpiCmdEx()\n\r");

    /* Start the command */
    while(SPIM_LockCS(pSpid, pSdSpid->bCs) != SPIM_OK);

    pSdSpid->pCmd = pCommand;

    /* Wait the card to ready */
    error = _WaitBusy(pSdSpid);

    /* Process the command */
    if (error == SDSPI_SUCCESS) {

        /* Special commands */
        if (pCommand->cmdOp.bVal == SDMMC_CMD_POWERONINIT) {

        }
        /* Send command & get CResp */
        else if (pCommand->cmdOp.bmBits.sendCmd) {
            error = _SendCmd(pSdSpid);
        }

        /* Transfer data */
        if (error == SDSPI_SUCCESS) {
            if (pCommand->blockSize && pCommand->nbBlock) {
                if (pCommand->cmdOp.bmBits.xfrData == SDMMC_CMD_RX)
                    error = _ReadBlocks(pSdSpid);
                else if (pCommand->cmdOp.bmBits.xfrData == SDMMC_CMD_TX)
                    error = _WriteBlocks(pSdSpid);
            }
        }

        /* Stop token */
        if (pCommand->cmdOp.bVal == SDMMC_CMD_STOPTOKEN) {
            uint8_t stopToken = SDSPI_STOP_TRAN;
            error = _SyncBytes(pSdSpid, &stopToken, 1);
            while(_WaitBusy(pSdSpid) == SDSPI_ERROR_TIMEOUT);
        }
    }

    /* End the command */
    pCommand->status = error;
    pSdSpid->pCmd = 0;
    SPIM_ReleaseCS(pSpid);
    //while(SPIM_ReleaseCS(pSpid) != SPIM_OK);

    if (pCommand->callback) pCommand->callback(error, pCommand);

    return error;
}

