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
 * \addtogroup external_component External Component
 *
 * \addtogroup at25d_module AT25 driver
 * \ingroup external_component
 * The AT25 serial dataflash driver is based on the corresponding AT25 SPI driver.
 * A AT25 instance has to be initialized using the Dataflash levle function
 * AT25_Configure(). AT25 Dataflash can be automatically detected using
 * the AT25_FindDevice() function. Then AT25 dataflash operations such as
 * read, write and erase DF can be launched using AT25_SendCommand function
 * with corresponding AT25 command set.
 *
 * \section Usage
 * <ul>
 * <li> Reads a serial flash device ID using AT25D_ReadJedecId().</li>
 * <li> Reads data from the At25 at the specified address using AT25D_Read().</li>
 * <li> Writes data on the At25 at the specified address using AT25D_Write().</li>
 * <li> Erases all chip using AT25D_EraseBlock().</li>
 * <li> Erases a specified block using AT25D_EraseBlock().</li>
 * <li> Poll until the At25 has completed of corresponding operations using
 * AT25D_WaitReady().</li>
 * <li> Retrieves and returns the At25 current using AT25D_ReadStatus().</li>
 * </ul>
 *
 * Related files :\n
 * \ref at25d.c\n
 * \ref at25d.h.\n
 */
/*@{*/
/*@}*/

/**
 * \file
 *
 * Implementation for the AT25 Serialflash driver.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <assert.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/** SPI clock frequency used in Hz. */
#define SPCK            BOARD_SF_SPCK

/** SPI chip select configuration value. */
#define CSR             (SPI_CSR_NCPHA | \
                         SPI_CSR_CSAAT | \
                         SPID_CSR_DLYBCT(BOARD_MCK, 100) | \
                         SPID_CSR_DLYBS(BOARD_MCK, 5) | \
                         SPID_CSR_SCBR(BOARD_MCK, SPCK))


/** Device ready/busy status bit. */
#define AT25_STATUS_RDYBSY          (1 << 0)
/** Device is ready. */
#define AT25_STATUS_RDYBSY_READY    (0 << 0)
/** Device is busy with internal operations. */
#define AT25_STATUS_RDYBSY_BUSY     (1 << 0)
/** Write enable latch status bit. */
#define AT25_STATUS_WEL             (1 << 1)
/** Device is not write enabled. */
#define AT25_STATUS_WEL_DISABLED    (0 << 1)
/** Device is write enabled. */
#define AT25_STATUS_WEL_ENABLED     (1 << 1)
/** Software protection status bitfield. */
#define AT25_STATUS_SWP             (3 << 2)
/** All sectors are software protected. */
#define AT25_STATUS_SWP_PROTALL     (3 << 2)
/** Some sectors are software protected. */
#define AT25_STATUS_SWP_PROTSOME    (1 << 2)
/** No sector is software protected. */
#define AT25_STATUS_SWP_PROTNONE    (0 << 2)
/** Write protect pin status bit. */
#define AT25_STATUS_WPP             (1 << 4)
/** Write protect signal is not asserted. */
#define AT25_STATUS_WPP_NOTASSERTED (0 << 4)
/** Write protect signal is asserted. */
#define AT25_STATUS_WPP_ASSERTED    (1 << 4)
/** Erase/program error bit. */
#define AT25_STATUS_EPE             (1 << 5)
/** Erase or program operation was successful. */
#define AT25_STATUS_EPE_SUCCESS     (0 << 5)
/** Erase or program error detected. */
#define AT25_STATUS_EPE_ERROR       (1 << 5)
/** Sector protection registers locked bit. */
#define AT25_STATUS_SPRL            (1 << 7)
/** Sector protection registers are unlocked. */
#define AT25_STATUS_SPRL_UNLOCKED   (0 << 7)
/** Sector protection registers are locked. */
#define AT25_STATUS_SPRL_LOCKED     (1 << 7)

/** Read array command code. */
#define AT25_READ_ARRAY             0x0B
/** Read array (low frequency) command code. */
#define AT25_READ_ARRAY_LF          0x03
/** Block erase command code (4K block). */
#define AT25_BLOCK_ERASE_4K         0x20
/** Block erase command code (32K block). */
#define AT25_BLOCK_ERASE_32K        0x52
/** Block erase command code (64K block). */
#define AT25_BLOCK_ERASE_64K        0xD8
/** Chip erase command code 1. */
#define AT25_CHIP_ERASE_1           0x60
/** Chip erase command code 2. */
#define AT25_CHIP_ERASE_2           0xC7
/** Byte/page program command code. */
#define AT25_BYTE_PAGE_PROGRAM      0x02
/** Sequential program mode command code 1. */
#define AT25_SEQUENTIAL_PROGRAM_1   0xAD
/** Sequential program mode command code 2. */
#define AT25_SEQUENTIAL_PROGRAM_2   0xAF
/** Write enable command code. */
#define AT25_WRITE_ENABLE           0x06
/** Write disable command code. */
#define AT25_WRITE_DISABLE          0x04
/** Protect sector command code. */
#define AT25_PROTECT_SECTOR         0x36
/** Unprotect sector command code. */
#define AT25_UNPROTECT_SECTOR       0x39
/** Read sector protection registers command code. */
#define AT25_READ_SECTOR_PROT       0x3C
/** Read status register command code. */
#define AT25_READ_STATUS            0x05
/** Write status register command code. */
#define AT25_WRITE_STATUS           0x01
/** Read manufacturer and device ID command code. */
#define AT25_READ_JEDEC_ID          0x9F
/** Deep power-down command code. */
#define AT25_DEEP_PDOWN             0xB9
/** Resume from deep power-down command code. */
#define AT25_RES_DEEP_PDOWN         0xAB

/*----------------------------------------------------------------------------
 *        Static variables
 *----------------------------------------------------------------------------*/

/** Array of recognized serial firmware dataflash chips. */
static const At25Desc at25Devices[] = {
    /* name,        Jedec ID,       size,  page size, block size, block erase command */
    {"AT25DF041A" , 0x0001441F,      512 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT25DF161"  , 0x0002461F, 2 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT26DF081A" , 0x0001451F, 1 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT26DF0161" , 0x0000461F, 2 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT26DF161A" , 0x0001461F, 2 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT25DF321" ,  0x0000471F, 4 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT25DF321A" , 0x0001471F, 4 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT25DF512B" , 0x0001651F,       64 * 1024, 256, 32 * 1024, AT25_BLOCK_ERASE_32K},
    {"AT25DF512B" , 0x0000651F,       64 * 1024, 256, 32 * 1024, AT25_BLOCK_ERASE_32K},
    {"AT25DF021"  , 0x0000431F,      256 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"AT26DF641" ,  0x0000481F, 8 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    /* Manufacturer: ST */
    {"M25P05"     , 0x00102020,       64 * 1024, 256, 32 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P10"     , 0x00112020,      128 * 1024, 256, 32 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P20"     , 0x00122020,      256 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P40"     , 0x00132020,      512 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P80"     , 0x00142020, 1 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P16"     , 0x00152020, 2 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P32"     , 0x00162020, 4 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"M25P64"     , 0x00172020, 8 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    /* Manufacturer: Windbond */
    {"W25X10"     , 0x001130EF,      128 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"W25X20"     , 0x001230EF,      256 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"W25X40"     , 0x001330EF,      512 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"W25X80"     , 0x001430EF, 1 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    /* Manufacturer: Macronix */
    {"MX25L512"   , 0x001020C2,       64 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"MX25L3205"  , 0x001620C2, 4 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    {"MX25L6405"  , 0x001720C2, 8 * 1024 * 1024, 256, 64 * 1024, AT25_BLOCK_ERASE_64K},
    /* Other */
    {"SST25VF512" , 0x000048BF,       64 * 1024, 256, 32 * 1024, AT25_BLOCK_ERASE_32K}
};

/** Number of recognized serialflash. */
#define NUMDATAFLASH    (sizeof(at25Devices) / sizeof(At25Desc))

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
 static EAt25Status AT25_SendCommand(
     At25 *pAt25,
     At25Cmd *pAt25Cmd)

 {
     /* Shortcut to SPI  driver */
     Spi *pSpi;

     /* Temporary variables used to transfer information with lower level drivers */
     /* These variables are located in the stack. This is ok since the function */
     /* does not return before the end of the spi operation or the operation is */
     /* not pre-empted */
     /* In multi-tasking environment, it is recommended to use malloc functions */
     ESpimStatus spiStatus;
     uint32_t cmdBuffer[2];

     /* Sanity check */
     assert(pAt25 != NULL);
     assert(pAt25Cmd->cmdSize < 8);

     /* Create a shortcut to Spi master API */
     pSpi = pAt25->pSpi;
     assert(pSpi != NULL);

     /* TODO: this is the place to test a semaphore as upper layer tasks can do
      * concurrent access to the SPI hw */

     /* Enable Chip select corresponding to the serial flash */
     SPIM_LockCS( pSpi, pAt25->cs);

     /* Store command and address in command buffer */
     cmdBuffer[0] = (pAt25Cmd->cmd & 0x000000FF)
                            | ((pAt25Cmd->address & 0x0000FF) << 24)
                            | ((pAt25Cmd->address & 0x00FF00) << 8)
                            | ((pAt25Cmd->address & 0xFF0000) >> 8);

     /* Transfer the command */
     spiStatus = SPIM_TransferBuffer(pSpi, (uint16_t *)cmdBuffer, (uint16_t *)cmdBuffer, pAt25Cmd->cmdSize);
     if (spiStatus != SPIM_OK) {
    	 SPIM_ReleaseCS( pSpi);
    	 return AT25_ERROR_SPI;
     }
     /* Transfer data buffer */
     spiStatus = SPIM_TransferBuffer(pSpi, (uint16_t *) pAt25Cmd->pData, (uint16_t *)pAt25Cmd->pData, pAt25Cmd->dataSize);

     /* Disable chip select */
     SPIM_ReleaseCS( pSpi);

     if (spiStatus != SPIM_OK) {
    	 return AT25_ERROR_SPI;
     }
     return (AT25_SUCCESS);
 }

 /*----------------------------------------------------------------------------
  *        Global functions
  *----------------------------------------------------------------------------*/
/**
 * \brief Initialize At25 object.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 * \param pSpim  Pointer to SPI master driver
 * \param cs     Active chip select for the connect flash device
 */
extern EAt25Status AT25_Initialize(At25 *pAt25, Spi *pSpi, uint8_t cs)
{   
    
    /* Initialize At25 object */
    memset(pAt25, 0, sizeof(At25));
    pAt25->pSpi = pSpi;
    pAt25->cs   = cs;
        
    /* Configure the SPI chip select for the serial flash */
    SPIM_ConfigureCS(pSpi, cs, CSR);
    
    return AT25_SUCCESS;
}

/**
 * \brief Reads and returns the status register of the serial flash.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 */
extern EAt25Status AT25_ReadStatus(At25 *pAt25, uint8_t *pAt25Status)
{
    EAt25Status status;
    At25Cmd at25Cmd;
    uint32_t dwAt25Status;
    
    assert(pAt25);

    /* Issue a read ID command */
    at25Cmd.cmd      = AT25_READ_STATUS;
    at25Cmd.cmdSize  = 1;
    at25Cmd.pData    = &dwAt25Status;
    at25Cmd.dataSize = 1;
    at25Cmd.address  = 0;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    *pAt25Status = (uint8_t) dwAt25Status;
    
    return status;
}

/**
 * \brief Writes the given value in the status register of the serial flash device.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 * \param status  Status to write.
 */
extern EAt25Status AT25_WriteStatus(At25 *pAt25, uint8_t at25Status)
{
    EAt25Status status;
    At25Cmd at25Cmd;
    uint32_t dwAt25Status = at25Status;

    assert(pAt25);

    /* Issue a write status command */
    at25Cmd.cmd      = AT25_WRITE_STATUS;
    at25Cmd.cmdSize  = 1;
    at25Cmd.pData    = &dwAt25Status;
    at25Cmd.dataSize = 1;
    at25Cmd.address  = 0;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    
    return status;
}

/**
 * \brief  Waits for the serial flash device to become ready to accept new commands.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 */
extern EAt25Status AT25_WaitReady(At25 *pAt25)
{
    EAt25Status status;
    uint8_t at25StatusReg;
    unsigned char ready = 0;

    assert(pAt25);

    /* Read status register and check busy bit */
    while (!ready) {
        status = AT25_ReadStatus(pAt25, &at25StatusReg);
        if (status != AT25_SUCCESS)
            return status;
        /* Immediately return if the device is already un-protected */
        if ((at25StatusReg & AT25_STATUS_RDYBSY) == AT25_STATUS_RDYBSY_READY) {
            ready = 1;
        }
    }
    return AT25_SUCCESS;
}

/**
 * \brief Reads and returns the serial flash device ID.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 */
extern EAt25Status AT25_ReadJedecId(At25 *pAt25, uint32_t *pJedecId)
{
    EAt25Status status;
    At25Cmd at25Cmd;

    assert(pAt25);

    /* Issue a read ID command */
    at25Cmd.cmd      = AT25_READ_JEDEC_ID;
    at25Cmd.cmdSize  = 1;
    at25Cmd.pData    = pJedecId;
    at25Cmd.dataSize = 3;
    at25Cmd.address  = 0;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    *pJedecId &= 0x00FFFFFF;
    return status;
}

/**
 * \brief Enables critical writes operation on a serial flash device, such as sector
 * protection, status register, etc.
 *
 * \para pAt25  Pointer to an AT25 driver instance.
 *
 * \return AT25_SUCCESS if the device has been unprotected; otherwise returns
 * AT25_ERROR_PROTECTED.
 */
extern EAt25Status AT25_EnableWrite(At25 *pAt25)
{
    EAt25Status status;
    At25Cmd at25Cmd;

    /* Sanity check */
    assert(pAt25);

    /* Issue a write enable command */
    at25Cmd.cmd      = AT25_WRITE_ENABLE;
    at25Cmd.cmdSize  = 1;
    at25Cmd.pData    = NULL;
    at25Cmd.dataSize = 0;
    at25Cmd.address  = 0;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    return status;
}

/**
 * \brief Unprotects the contents of the serial flash device.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 *
 * \return AT25_SUCCESS if the device has been unprotected; otherwise returns
 * AT25_ERROR_PROTECTED.
 */
extern EAt25Status AT25_Unprotect(At25 *pAt25)
{
    EAt25Status status;
    uint8_t at25StatusReg;
 
    /* Sanity check */
    assert(pAt25);

    /* Get the status register value to check the current protection */
    /* Check that the flash is unprotected */
    status = AT25_ReadStatus(pAt25, &at25StatusReg);
    if (status != AT25_SUCCESS)
        return status;
    /* Immediately return if the device is already un-protected */
    if ((at25StatusReg & AT25_STATUS_SWP) == AT25_STATUS_SWP_PROTNONE) {
        return AT25_SUCCESS;
    }
 
    /* Perform a global unprotect command */
    status = AT25_EnableWrite(pAt25);
    if (status != AT25_SUCCESS)
        return status;

    status = AT25_WriteStatus(pAt25, 0);
    if (status != AT25_SUCCESS)
        return status;

    /* Check the new status */
    status = AT25_ReadStatus(pAt25, &at25StatusReg);
    if (status != AT25_SUCCESS)
        return status;
    if ((at25StatusReg & (AT25_STATUS_SPRL | AT25_STATUS_SWP)) != 0) {
        return AT25_ERROR_PROTECTED;
    }
    
    return AT25_SUCCESS;
}

/**
 * \brief Erases all the content of the memory chip.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 *
 * \return AT25_SUCCESS if the device has been unprotected; otherwise returns
 * AT25_ERROR_PROTECTED.
 */
extern EAt25Status AT25_EraseChip(At25 *pAt25)
{
    EAt25Status status;
    uint8_t at25StatusReg;
    At25Cmd at25Cmd;

    /* Sanity check */
    assert(pAt25);

    /* Check that the flash is unprotected */
    status = AT25_ReadStatus(pAt25, &at25StatusReg);
    if (status != AT25_SUCCESS)
        return status;
    if ((at25StatusReg & AT25_STATUS_SWP) != AT25_STATUS_SWP_PROTNONE) {
        return AT25_ERROR_PROTECTED;
    }

    /* Enable critical write operation */
    status = AT25_EnableWrite(pAt25);
    if (status != AT25_SUCCESS)
        return status;

    /* Erase the chip */
    at25Cmd.cmd      = AT25_CHIP_ERASE_2;
    at25Cmd.cmdSize  = 1;
    at25Cmd.pData    = NULL;
    at25Cmd.dataSize = 0;
    at25Cmd.address  = 0;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    if (status != AT25_SUCCESS)
        return status;

    /* Wait for transfer to finish */
    status = AT25_WaitReady(pAt25);
    return status;
}

/**
 *\brief  Erases the specified 64KB block of the serial firmware dataflash.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 * \param address  Address of the block to erase.
 *
 * \return 0 if successful; otherwise returns AT25_ERROR_PROTECTED if the
 * device is protected or AT25_ERROR_BUSY if it is busy executing a command.
 */
extern EAt25Status AT25_EraseBlock(At25 *pAt25, uint32_t address)
{
    EAt25Status status;
    uint8_t at25StatusReg;
    At25Cmd at25Cmd;

    /* Sanity check */
    assert(pAt25);
    
    if (pAt25->pDesc == NULL) {
        return AT25_ERROR_INIT;
    }

    /* Check that the flash is ready and unprotected */
    status = AT25_ReadStatus(pAt25, &at25StatusReg);
    if (status != AT25_SUCCESS)
        return status;
    if ((at25StatusReg & AT25_STATUS_RDYBSY) != AT25_STATUS_RDYBSY_READY) {
        return AT25_ERROR_BUSY;
    }
    else if ((at25StatusReg & AT25_STATUS_SWP) != AT25_STATUS_SWP_PROTNONE) {
        return AT25_ERROR_PROTECTED;
    }

    /* Enable critical write operation */
    status = AT25_EnableWrite(pAt25);
    if (status != AT25_SUCCESS)
        return status;
    

    /* Start the block erase command */
    at25Cmd.cmd      = AT25_BlockEraseCmd(pAt25);
    at25Cmd.cmdSize  = 4;
    at25Cmd.pData    = NULL;
    at25Cmd.dataSize = 0;
    at25Cmd.address  = address;
    status = AT25_SendCommand(pAt25, &at25Cmd);
    if (status != AT25_SUCCESS)
        return status;

    /* Wait for transfer to finish */
    status = AT25_WaitReady(pAt25);
    if (status != AT25_SUCCESS)
        return status;
 
    return AT25_SUCCESS;
}

/**
 * \brief Writes data at the specified address on the serial firmware dataflash. The
 * page(s) to program must have been erased prior to writing. This function
 * handles page boundary crossing automatically.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 * \param pData  Data buffer.
 * \param size  Number of bytes in buffer.
 * \param address  Write address.
 *
 * \return AT25_SUCCESS if successful; otherwise, returns AT25_WRITE_ERROR is there has
 * been an error during the data programming.
 */
extern EAt25Status AT25_Write(
    At25 *pAt25,
    uint32_t *pData,
    uint16_t size,
    uint32_t address)
{
    unsigned int pageSize;
    unsigned int writeSize;
    EAt25Status status;
    uint8_t at25StatusReg;
    At25Cmd at25Cmd;

    /* Sanity check */
    assert(pAt25);
    assert(pData);

    /* Retrieve device page size */
    pageSize = AT25_PageSize(pAt25);

    /* Program one page after the other */
    while (size > 0) {
        /* Compute number of bytes to program in page */
        writeSize = min(size, pageSize - (address % pageSize));

        /* Enable critical write operation */
        AT25_EnableWrite(pAt25);

        at25Cmd.cmd      = AT25_BYTE_PAGE_PROGRAM;
        at25Cmd.cmdSize  = 4;
        at25Cmd.pData    = pData;
        at25Cmd.dataSize = writeSize;
        at25Cmd.address  = address;

        /* Program page */
        status = AT25_SendCommand(pAt25, &at25Cmd);
        if (status != AT25_SUCCESS)
            return status;

        /* Poll the Serial flash status register until the operation is achieved */
        status = AT25_WaitReady(pAt25);
        if (status != AT25_SUCCESS)
            return status;

        /* Make sure that write was without error */
        status = AT25_ReadStatus(pAt25, &at25StatusReg);
        if (status != AT25_SUCCESS)
            return status;
        if ((at25StatusReg & AT25_STATUS_EPE) == AT25_STATUS_EPE_ERROR) {
            return AT25_ERROR_WRITE;
        }

        pData += writeSize;
        size -= writeSize;
        address += writeSize;
    }

    return AT25_SUCCESS;
}

/**
 * \brief Reads data from the specified address on the serial flash.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 * \param pData  Data buffer.
 * \param size  Number of bytes to read.
 * \param address  Read address.
 *
 * \return AT25_SUCCESS if successful; otherwise, fail.
 */
extern EAt25Status AT25_Read(
    At25 *pAt25,
    uint32_t *pData,
    uint16_t size,
    uint32_t address)
{
    EAt25Status result;
    At25Cmd at25Cmd;
 
    /* Sanity check */
    assert (pAt25);
    
    /* Initialize a Read command to be sent through SPI */
    at25Cmd.cmd      = AT25_READ_ARRAY_LF;
    at25Cmd.cmdSize  = 4;
    at25Cmd.pData    = pData;
    at25Cmd.dataSize = size;
    at25Cmd.address  = address;
    
    /* Check read size */
    if ( pAt25->pDesc && (AT25_Size(pAt25) < (address + size)) ) {
        return AT25_ERROR_OUT_OF_BOUND;
    }
    
    /* Start a read operation */
    result = AT25_SendCommand(pAt25, &at25Cmd);
 
    return result;
}

/**
 * \brief Tries to detect a serial firmware flash device given its JEDEC identifier.
 * Initialize the pAt25->pDesc pointer.
 * The JEDEC id can be retrieved by sending the correct command to the device.
 *
 * \param pAt25  Pointer to an AT25 driver instance.
 *
 * \return AT25_SUCCESS if successful; otherwise, fail, pAt25->pDesc pointer is NULL.
 */
extern EAt25Status AT25_GetDeviceDescriptor(At25 *pAt25)
{
    EAt25Status status;
    uint32_t dwJedecId;
    unsigned int i = 0;

    assert(pAt25);

    pAt25->pDesc = NULL;

    /* Read Serial flash JDEC identifier */
    status = AT25_ReadJedecId(pAt25, &dwJedecId);
    if (status != AT25_SUCCESS)
        return status;
    
    /* Search if device is recognized */
    status = AT25_ERROR_INIT;
    while ((i < NUMDATAFLASH) && !(pAt25->pDesc)) {

        if (dwJedecId == at25Devices[i].jedecId) {
            pAt25->pDesc = &(at25Devices[i]);
            status = AT25_SUCCESS;
        }

        i++;
    }
    
    return status;
}
