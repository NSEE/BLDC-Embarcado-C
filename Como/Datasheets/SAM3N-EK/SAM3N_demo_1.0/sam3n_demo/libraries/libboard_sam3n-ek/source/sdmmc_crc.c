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

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/**< STD CCITT Polynomial: x^16 + x^12 + x^5 + 1 */
#define Poly_16_CCITT       0x1021
/**< CRC7 Polynomial for SD/MMC: x^7 + x^3 + 1 */
#define Poly_7              0x89

/**< CRC7 table initialized */
#define CRC7_initialized        (0x1)
/**< CRC16 CCITT table initialized */
#define CRC16CCITT_initialized  (0x1<<1)

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/**< Table for CRC-7 0x89 (polynomial x^7 + x^3 + 1) */
static uint8_t  crc7_Table[256];
/**< Table for CRC-16 CCITT 0x0x1021 (x^16 + x^12 + x^15 + 1) */
static uint16_t crc16CCITT_Table[256];

/**< Initialize status for CRC tables */
static uint8_t  crcInitFlags = 0;

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * Calculate CRC7 Table (x^7 + x^3 + 1)
 */
static void crc7_Table_init(void)
{
    uint32_t i, j;
    for (i = 0; i < 256; i ++) {
        crc7_Table[i] = (i & 0x80) ? i ^ Poly_7 : i;
        for (j = 7; j; j --) {
            crc7_Table[i] <<= 1;
            if (crc7_Table[i] & 0x80)
                crc7_Table[i] ^= Poly_7;
        }
    }
    crcInitFlags |= CRC7_initialized;
}
/**
 * Calculate CRC16 Table (x^16 + x^12 + x^5 + 1)
 */
static void crc16CCITT_Table_init(void)
{
    uint32_t i, j;
    uint16_t crc, c;
    for (i = 0; i < 256; i ++) {
        crc = 0; c = ((uint16_t)i << 8);
        for (j = 8; j; j --) {
            if ( (crc ^ c) & 0x8000 ) crc   = ( crc << 1 ) ^ Poly_16_CCITT;
            else                      crc   =   crc << 1;
            c <<= 1;
        }
        crc16CCITT_Table[i] = crc;
    }
    crcInitFlags |= CRC16CCITT_initialized;
}

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 * Calculate CRC7 (x^7 + x^3 + 1)
 * \param pData Pointer to data block.
 * \param dwCnt Data block size.
 * \return CRC7
 */
uint8_t crc7_calc(uint8_t *pData, uint32_t dwCnt)
{
    uint32_t i;
    uint8_t crc = 0;

    if ((crcInitFlags & CRC7_initialized) == 0)
        crc7_Table_init();

    for (i = 0; i < dwCnt; i ++) {
        crc = crc7_Table[(crc << 1) ^ pData[i]];
    }

    return crc;
}

/**
 * Calculate CRC16 CCITT (x^16 + x^12 + x^5 +1)
 * \param pData Pointer to data block.
 * \param dwCnt Data block size.
 * \return CRC16 CCITT
 */
uint16_t crc16_ccitt_calc(uint8_t *pData, uint32_t dwCnt)
{
    uint32_t i;
    uint16_t crc = 0, tmp;

    if ((crcInitFlags & CRC16CCITT_initialized) == 0)
        crc16CCITT_Table_init();

    for (i = 0; i < dwCnt; i ++) {
        tmp =  crc       ^ (0x00ff & (uint16_t)pData[i]);
        crc = (crc >> 8) ^ crc16CCITT_Table[ tmp & 0xff ];
    }

    return crc;
}

